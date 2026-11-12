#pragma once
// ---------------------------------------------------------------------------
// SmartGps.h - T1000-E smart GPS manager (experimental, based on
// MeshCore companion-v1.18.0).
//
// Adds board CLI commands (via MainBoard::handleCommand hook):
//   gps.mode on|off|auto   - select GPS operating mode (RAM only)
//   gps.still <30..3600>   - AUTO stillness timeout in seconds (RAM only)
//   gps.th <0..255>        - any-motion threshold, 7.8125 mg/count at +/-4G
//   gps.status             - report mode / GPS power / IMU / timeout / th / mot
//   gps.motion             - QMA6100P register readback + counters (diagnostic)
//
// Design notes:
//  - All SmartGps settings are VOLATILE (RAM only, by design for v1).
//  - The stock App GPS setting (CMD_SET_CUSTOM_VAR -> setSettingValue("gps"))
//    always wins: it cancels AUTO and re-syncs this state machine
//    (onManualGpsSetting). Boot-time applyGpsPrefs() goes through the same
//    path, so mode_ tracks the persisted setting after reboot.
//  - GPS power is driven exclusively through T1000SensorManager
//    (start_gps()/stop_gps()), so gps_active and NMEA handling stay in sync.
//
// QMA6100P register configuration verified against THREE independent
// sources: (1) QMA6100P Preliminary Datasheet Rev A1 (QST, register map
// pp.25-35), (2) Seeed's official T1000-E firmware driver
// (Seeed-Tracker-T1000-E-for-LoRaWAN-dev-board t1000_e/peripherals/src/
// qma6100p.c, qma6100p_motion_init()), (3) Meshtastic pinned driver
// (QMA6100P_Arduino_Library @ 14c900b) + RIOT OS qma6100p_regs.h.
// The SparkFun-derived bitfield structs in QMA6100P_regs.h are byte-MIRRORED
// vs the datasheet tables; where they disagree, datasheet + Seeed win:
//   * INT_ST0 (0x09): any-motion axis flags are bits 2:0 (X=bit0, Y=bit1,
//     Z=bit2) -> mask 0x07. Seeed AMD_X/Y/Z_BIT = 0x01/0x02/0x04; Meshtastic
//     polls (status & 7). NO_MOT is bit7, not bit0.
//   * INT_MAP1 (0x1A): INT1_ANY_MOT = bit0 (0x01); INT1_NO_MOT = bit7.
//     Seeed writes 0x81 = "any motion int map, no motion int map".
//     v1.0 wrote 0x80 (mapping NO-motion only) - FIXED in v1.1.
//   * INTPIN_CONF (0x20): bit0=INT1_LVL (1=active-high), bit1=INT1_OD
//     (1=open-drain); default 0x05 = push-pull ACTIVE-HIGH (RIOT confirms).
//     Meshtastic's |=0x02 -> 0x07 = open-drain active-high, so its
//     FALLING-edge attach never fires for motion. v1.1 clears bits1:0 ->
//     push-pull ACTIVE-LOW, matching our FALLING + INPUT_PULLUP wiring.
//   * MOT_CONF2 (0x2E) ANY_MOT_TH IS documented, default 0x00: threshold =
//     ANY_MOT_TH * 16 * range-LSB (Slope input; 0x2F default 0). At +/-4G
//     (LSB 488 ug) one count = 7.8125 mg; default 0 triggers on sensor
//     noise. v1.1 writes QMA_MOT_TH_DEFAULT (=8 -> 62.5 mg), tunable at
//     runtime via gps.th. Seeed clamps the equivalent to 10..2000 mg.
//     NOTE: this is a well-founded STARTING value, not a field-validated
//     setting - real tuning requires hardware tests (report item H03).
//   * MOT_CONF0 (0x2C) bits1:0 = ANY_MOT_DUR: trigger requires slope > th
//     for (ANY_MOT_DUR+1) consecutive samples; default 0 = 1 sample.
//     v1.1 writes 3 (4 samples ~40 ms at 100 Hz) like Seeed (their 0x8b =
//     any-dur 3 | no-mot-dur 120 s; we do not use the no-motion feature).
//   * BW (0x10) v1.1 writes 0x00 = 100 Hz ODR, filters off (Seeed init
//     writes QMA6100P_BW_100=0; datasheet default 0xE0 has the same BW
//     bits = 100 Hz plus HPF div800, so this only makes state explicit).
//   * INT_CFG (0x21) |= 0x81 keeps LATCH_INT (default 1) and adds
//     INT_RD_CLR, so tick()'s read of 0x09 clears the latch and re-arms
//     the INT pin for the next event.
//
// Phase status:
//   Phase 1 (done): gps.mode on/off + CLI wiring.
//   Phase 2 (done): QMA6100P power-up, init, motion IRQ capture + counting.
//   Phase 3 (done): AUTO GPS on/off closed loop in tick().
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Sensor power rail and interrupt pin. Prefer the board's variant.h macros;
// fall back to the values confirmed in variants/t1000-e/variant.h
// (PIN_3V3_ACC_EN=39/P1.07, QMA_6100P_INT_PIN=34/P1.02).
#ifndef PIN_3V3_ACC_EN
#define PIN_3V3_ACC_EN 39
#endif
#ifndef QMA_6100P_INT_PIN
#define QMA_6100P_INT_PIN 34
#endif

// QMA6100P constants verified from QMA6100P.h / QMA6100P_regs.h.
#define QMA_ADDR_LOW        0x12   // QMA6100P_ADDRESS_LOW
#define QMA_ADDR_HIGH       0x13   // QMA6100P_ADDRESS_HIGH
#define QMA_CHIP_ID_REG     0x00
#define QMA_CHIP_ID_VAL     0x90   // QMA6100P_CHIP_ID
#define QMA_REG_SR          0x36   // soft reset
#define QMA_REG_FSR         0x0F   // range / filter
#define QMA_REG_BW          0x10   // ODR / filter config (0x00 = 100 Hz)
#define QMA_REG_PM          0x11   // power mode
#define QMA_REG_INT_EN2     0x18   // any-motion enable (XYZ, bits 2:0)
#define QMA_REG_INT_MAP1    0x1A   // map any-motion -> INT1 (bit0!)
#define QMA_REG_INTPINT     0x20   // INT pin electrical config
#define QMA_REG_INT_CFG     0x21   // INT latch config
#define QMA_REG_INT_ST0     0x09   // interrupt status (read clears latch)
#define QMA_REG_MOT_CONF0   0x2C   // bits1:0 = ANY_MOT_DUR (dur+1 samples)
#define QMA_REG_MOT_CONF2   0x2E   // ANY_MOT_TH: th * 16 * LSB (Slope mode)
#define QMA_INT_ST0_ANY_MOT 0x07   // INT_ST0 bits2:0 = any-motion axis flags
#define QMA_MOT_DUR_4       0x03   // ANY_MOT_DUR=3 -> 4 samples (~40 ms)
#define QMA_MOT_TH_DEFAULT  8      // 8 * 7.8125 mg = 62.5 mg at +/-4G range

class SmartGps {
public:
  enum Mode : uint8_t { OFF, ON, AUTO };
  typedef void (*GpsPowerFn)(bool);

  explicit SmartGps(GpsPowerFn power) : power_(power) {}

  // Called from T1000SensorManager::setSettingValue("gps", ...) AFTER the
  // manager already drove the hardware (stock App toggle, stock boot prefs,
  // or any future manual path). Manual setting has top priority: it cancels
  // AUTO and shuts the motion sensor down.
  void onManualGpsSetting(bool enabled) {
    disableImu();
    mode_ = enabled ? ON : OFF;
    gps_on_ = enabled;
  }

  bool setMode(Mode wanted, uint32_t now) {
    if (wanted == AUTO) {
      // Never enter AUTO if the accelerometer cannot be brought up.
      if (!imu_ready_ && !enableImu()) return false;
      mode_ = AUTO;
      last_motion_ms_ = now;
      gps_since_ms_ = now;
      motion_seen_ = false;
      // AUTO starts with GPS asleep until the first motion event (Phase 3
      // closes the loop; in Phase 2 this just leaves GPS off and counts IRQs).
      driveGps(false);
    } else {
      disableImu();
      mode_ = wanted;
      driveGps(wanted == ON);
    }
    return true;
  }

  // Called from T1000SensorManager::loop() (main-loop context).
  void tick(uint32_t now) {
    if (mode_ != AUTO || !imu_ready_) return;

    if (pendingIrq()) {
      // Read INT_ST0: with INT_RD_CLR=1 this clears the latch and lets the
      // INT pin return high, re-arming the next FALLING edge. Bits 2:0 name
      // the first axis that saw any-motion (datasheet order, mask 0x07).
      uint8_t status = 0;
      if (readReg(QMA_REG_INT_ST0, status)) {
        if (status & QMA_INT_ST0_ANY_MOT) noteMotion(now);
        else spurious_count_++;  // pin fired without any-motion status
      } else {
        // I2C read failed: the latch was NOT cleared (INT still held low)
        // and the event must not be lost. Count it as motion (fail toward
        // waking GPS) and, if the pin still reads low, requeue so the next
        // tick retries the status read/clear and re-arms the edge.
        noteMotion(now);
        if (digitalRead(QMA_6100P_INT_PIN) == LOW) irqFlag() = true;
      }
    }

    // Sleep GPS once it has been on long enough AND the device has been
    // still for the whole stillness window. Unsigned subtraction is
    // millis()-wraparound safe. A new motion IRQ above resets last_motion_ms_,
    // so repeated movement keeps deferring the sleep.
    if (gps_on_ && motion_seen_ &&
        elapsed(now, last_motion_ms_) >= still_ms_ &&
        elapsed(now, gps_since_ms_) >= min_on_ms_) {
      driveGps(false);
      motion_seen_ = false;      // require fresh motion to wake again
    }
  }

  // MainBoard::handleCommand hook. Returns false for commands outside the
  // gps.* family so the stock CLI keeps handling everything else.
  // 'reply' buffer is at least 163 bytes (MyMesh reply_buf[166] minus the
  // optional 3-char command prefix), replies below stay well under that.
  bool command(const char* cmd, uint32_t now, char* reply) {
    if (!cmd || !reply) return false;

    if (strcmp(cmd, "gps.status") == 0 || strcmp(cmd, "gps.mode") == 0) {
      writeStatus(reply);
      return true;
    }
    if (strcmp(cmd, "gps.mode on") == 0) {
      setMode(ON, now);
      strcpy(reply, "OK mode=on (RAM only)");
      return true;
    }
    if (strcmp(cmd, "gps.mode off") == 0) {
      setMode(OFF, now);
      strcpy(reply, "OK mode=off (RAM only)");
      return true;
    }
    if (strcmp(cmd, "gps.mode auto") == 0) {
      strcpy(reply, setMode(AUTO, now) ? "OK mode=auto (RAM only)"
                                       : "ERR QMA6100P not detected");
      return true;
    }
    if (strncmp(cmd, "gps.still ", 10) == 0) {
      unsigned long seconds = 0;
      char end = 0;
      // exactly one unsigned number, nothing trailing
      if (sscanf(cmd + 10, "%lu%c", &seconds, &end) == 1 &&
          seconds >= 30 && seconds <= 3600) {
        still_ms_ = (uint32_t)seconds * 1000UL;
        snprintf(reply, 120, "OK still=%lus (RAM only)", seconds);
      } else {
        strcpy(reply, "ERR usage: gps.still 30..3600");
      }
      return true;
    }
    if (strncmp(cmd, "gps.th ", 7) == 0) {
      unsigned int th = 0;
      char end = 0;
      // exactly one unsigned number 0..255, nothing trailing ("%u" on "-1"
      // parses as a huge unsigned and is rejected by the range check)
      if (sscanf(cmd + 7, "%u%c", &th, &end) == 1 && th <= 255) {
        if (imu_ready_) {
          if (writeReg(QMA_REG_MOT_CONF2, (uint8_t)th)) {
            mot_th_ = (uint8_t)th;
            snprintf(reply, 120, "OK th=%u applied (RAM only)", th);
          } else {
            snprintf(reply, 120, "ERR QMA write failed, th=%u kept",
                     (unsigned)mot_th_);
          }
        } else {
          mot_th_ = (uint8_t)th;   // applied by enableImu() on next auto
          snprintf(reply, 120, "OK th=%u stored (used on next auto)", th);
        }
      } else {
        strcpy(reply, "ERR usage: gps.th 0..255");
      }
      return true;
    }
    if (strcmp(cmd, "gps.motion") == 0) {
      if (!imu_ready_) {
        snprintf(reply, 140, "imu=off th=%u (arm with gps.mode auto)",
                 (unsigned)mot_th_);
      } else {
        // Readback of the motion config. Deliberately does NOT read 0x09:
        // with INT_RD_CLR=1 that would swallow a pending motion event
        // before tick() acts on it. All registers below are outside the
        // 0x09..0x0D clear range. int= shows the INT pin level directly
        // (expect 1 idle, 0 latched) for on-hardware polarity checks.
        uint8_t th = 0, dur = 0, bw = 0, fsr = 0, pin = 0;
        readReg(QMA_REG_MOT_CONF2, th);
        readReg(QMA_REG_MOT_CONF0, dur);
        readReg(QMA_REG_BW, bw);
        readReg(QMA_REG_FSR, fsr);
        readReg(QMA_REG_INTPINT, pin);
        snprintf(reply, 140,
                 "th=%u dur=%u bw=%u fsr=%u intpin=%u int=%d mot=%lu spur=%lu",
                 (unsigned)th, (unsigned)dur, (unsigned)bw, (unsigned)fsr,
                 (unsigned)pin, digitalRead(QMA_6100P_INT_PIN) ? 1 : 0,
                 (unsigned long)motion_count_, (unsigned long)spurious_count_);
      }
      return true;
    }
#ifdef SMARTGPS_TEST_INJECT
    // Host-test only hook; never compiled into firmware builds.
    if (strcmp(cmd, "gps.testmove") == 0) {
      irqFlag() = true;
      strcpy(reply, "OK fake motion queued");
      return true;
    }
#endif
    // Reserve the gps.* family: report usage instead of a generic
    // "Unknown command" for typos like "gps.mode oops".
    if (strncmp(cmd, "gps.mode", 8) == 0 || strncmp(cmd, "gps.still", 9) == 0 ||
        strncmp(cmd, "gps.status", 10) == 0 || strncmp(cmd, "gps.th", 6) == 0 ||
        strncmp(cmd, "gps.motion", 10) == 0) {
      strcpy(reply, "ERR usage: gps.mode on|off|auto, gps.still 30..3600, "
                    "gps.th 0..255, gps.status, gps.motion");
      return true;
    }
    return false;
  }

  Mode mode() const { return mode_; }
  bool gpsOn() const { return gps_on_; }
  bool imuReady() const { return imu_ready_; }
  uint32_t stillSeconds() const { return still_ms_ / 1000U; }
  uint32_t motionCount() const { return motion_count_; }
  uint8_t motionThreshold() const { return mot_th_; }
  uint32_t spuriousCount() const { return spurious_count_; }

private:
  static uint32_t elapsed(uint32_t now, uint32_t then) { return now - then; }

  // --- QMA6100P motion-sensor interrupt plumbing -------------------------
  static volatile bool& irqFlag() {
    static volatile bool flag = false;
    return flag;
  }
  static void motionIsr() { irqFlag() = true; }
  static bool pendingIrq() {
    noInterrupts();
    bool value = irqFlag();
    irqFlag() = false;
    interrupts();
    return value;
  }

  // --- I2C register helpers (single-byte) --------------------------------
  bool readReg(uint8_t reg, uint8_t& value) {
    Wire.beginTransmission(qma_addr_);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;   // repeated start
    if (Wire.requestFrom((int)qma_addr_, 1) != 1) return false;
    value = (uint8_t)Wire.read();
    return true;
  }
  bool writeReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(qma_addr_);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
  }
  bool updateReg(uint8_t reg, uint8_t clear_mask, uint8_t set_mask) {
    uint8_t val;
    return readReg(reg, val) &&
           writeReg(reg, (uint8_t)((val & ~clear_mask) | set_mask));
  }

  // Bring up the QMA6100P rail, verify chip ID, configure any-motion IRQ.
  bool enableImu() {
    pinMode(PIN_3V3_ACC_EN, OUTPUT);
    digitalWrite(PIN_3V3_ACC_EN, HIGH);   // power the sensor rail
    delay(20);                            // let the QMA6100P boot

    // Probe both possible addresses; require a valid chip ID.
    uint8_t id = 0;
    qma_addr_ = QMA_ADDR_LOW;
    if (!readReg(QMA_CHIP_ID_REG, id) || id != QMA_CHIP_ID_VAL) {
      qma_addr_ = QMA_ADDR_HIGH;
      if (!readReg(QMA_CHIP_ID_REG, id) || id != QMA_CHIP_ID_VAL) {
        digitalWrite(PIN_3V3_ACC_EN, LOW);
        return false;
      }
    }

    // Init sequence cross-verified against datasheet Rev A1 + Seeed official
    // driver + Meshtastic library (see the verification record at the top).
    bool ok = writeReg(QMA_REG_SR, 0xB6);        // soft reset
    delay(5);                                     // Seeed: hold reset 5 ms
    ok = ok && writeReg(QMA_REG_SR, 0x00);       // release reset
    delay(10);                                    // Seeed: boot time 10 ms
    ok = ok &&
      updateReg(QMA_REG_FSR, 0x0F, 0x02) &&      // range = +/-4G (488 ug LSB)
      writeReg(QMA_REG_BW, 0x00) &&              // 100 Hz ODR, filters off
      updateReg(QMA_REG_PM, 0x00, 0x80) &&       // PM.mode_bit = 1 (active)
      writeReg(QMA_REG_INT_EN2, 0x07) &&         // any-motion enable X|Y|Z
      writeReg(QMA_REG_MOT_CONF2, mot_th_) &&    // ANY_MOT_TH (x7.8125 mg)
      writeReg(QMA_REG_MOT_CONF0, QMA_MOT_DUR_4) && // slope>th for 4 samples
      updateReg(QMA_REG_INT_MAP1, 0x00, 0x01) && // INT1_ANY_MOT = bit0 (Seeed
                                                 // 0x81 comment + datasheet;
                                                 // v1.0's 0x80 = NO_MOT map!)
      updateReg(QMA_REG_INTPINT, 0x03, 0x00) &&  // INT1 push-pull ACTIVE-LOW
                                                 // (v1.0's |=0x02 left it
                                                 // active-high: no FALLING
                                                 // edge would ever come)
      updateReg(QMA_REG_INT_CFG, 0x00, 0x81);    // latch until cleared + rd-clear
    if (!ok) {
      digitalWrite(PIN_3V3_ACC_EN, LOW);
      return false;
    }
    delay(2);

    irqFlag() = false;
    pinMode(QMA_6100P_INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(QMA_6100P_INT_PIN), motionIsr, FALLING);
    // R1 fix (residual-latch window): motion between the latch-enable write
    // above and attachInterrupt (a few ms of I2C + delay(2) + pinMode) latches
    // INT low; no FALLING edge will ever arrive afterwards, leaving AUTO deaf
    // with imu=ok, mot=0. Sample the pin once after attaching: if it is
    // already low, queue the event ourselves. Race-free and idempotent -
    // motion arriving after attach takes the normal ISR path, and the next
    // tick() reads INT_ST0, clears the latch and re-arms the edge either way.
    if (digitalRead(QMA_6100P_INT_PIN) == LOW) irqFlag() = true;
    imu_ready_ = true;
    return true;
  }

  void disableImu() {
    if (imu_ready_) {
      detachInterrupt(digitalPinToInterrupt(QMA_6100P_INT_PIN));
      digitalWrite(PIN_3V3_ACC_EN, LOW);   // cut sensor rail to save power
      imu_ready_ = false;
    }
    irqFlag() = false;
  }

  void writeStatus(char* reply) {
    snprintf(reply, 140, "mode=%s gps=%s imu=%s still=%us th=%u mot=%lu",
             modeName(), gps_on_ ? "on" : "sleep",
             imu_ready_ ? "ok" : "off",
             (unsigned)(still_ms_ / 1000), (unsigned)mot_th_,
             (unsigned long)motion_count_);
  }
  const char* modeName() const {
    return mode_ == AUTO ? "auto" : (mode_ == ON ? "on" : "off");
  }
  // Shared motion-event bookkeeping (normal IRQ path and the I2C-failure
  // fallback both funnel through here).
  void noteMotion(uint32_t now) {
    motion_count_++;
    last_motion_ms_ = now;
    motion_seen_ = true;
    if (!gps_on_) {
      driveGps(true);          // motion woke GPS
      gps_since_ms_ = now;     // restart the min-on-time window
    }
  }
  // Only calls through on an actual transition, so repeated identical
  // commands never re-trigger the (blocking) GPS power sequence.
  void driveGps(bool enabled) {
    if (gps_on_ != enabled) {
      power_(enabled);
      gps_on_ = enabled;
    }
  }

  GpsPowerFn power_;
  Mode mode_ = OFF;          // matches initVariant(): GPS_EN=LOW at boot
  bool gps_on_ = false;      // matches T1000SensorManager::gps_active=false
  bool imu_ready_ = false;
  bool motion_seen_ = false;
  uint8_t qma_addr_ = QMA_ADDR_LOW;
  uint8_t mot_th_ = QMA_MOT_TH_DEFAULT;  // ANY_MOT_TH: 7.8125 mg/count @ +/-4G
  uint32_t motion_count_ = 0;
  uint32_t spurious_count_ = 0;          // IRQs without any-motion status bits
  uint32_t last_motion_ms_ = 0;
  uint32_t gps_since_ms_ = 0;
  uint32_t still_ms_ = 180000;   // default stillness timeout: 180 s
  uint32_t min_on_ms_ = 60000;   // minimum GPS on time in AUTO: 60 s
};

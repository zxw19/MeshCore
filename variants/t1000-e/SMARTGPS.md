# T1000-E Smart GPS (experimental)

Motion-triggered GPS power management for the Seeed SenseCAP T1000-E, built on
MeshCore `companion-v1.18.0` (commit `81dbae59`).

Status: **v1.1**. Host tests pass, and the AUTO on/off loop has been verified on
physical hardware. Still experimental; see *Known limitations*.

Release asset: `firmware.uf2` for the `t1000e_companion_radio_ble` environment.

## What it does

The QMA6100P accelerometer stays armed while the GNSS receiver is powered down.
The first any-motion interrupt wakes the GPS; the GPS goes back to sleep after
the device has been still for a configurable window (default 180 s) and has been
on for at least 60 s.

| Mode | GPS | QMA6100P | Notes |
|---|---|---|---|
| `off` | asleep | powered down | default at boot |
| `on` | always on | powered down | stock behaviour |
| `auto` | sleeps until motion | armed | closed loop in `tick()` |

Settings are **RAM only** and reset on reboot. The stock App setting (*Position
Settings*) always wins: toggling it calls `setSettingValue("gps", ...)`, which
cancels AUTO and re-syncs this state machine. Boot-time `applyGpsPrefs()` uses
the same path, so `mode=` follows the persisted App setting after a reboot.

## CLI

Send these from the MeshCore App (Settings > Command Line):

| Command | Example reply | Meaning |
|---|---|---|
| `gps.status` (or bare `gps.mode`) | `mode=auto gps=on imu=ok still=180s th=8 mot=7` | Mode, real GPS power, IMU state, stillness window, threshold, motion count |
| `gps.mode on\|off\|auto` | `OK mode=auto (RAM only)` | `auto` replies `ERR QMA6100P not detected` and keeps the previous mode if the sensor does not answer |
| `gps.still <30..3600>` | `OK still=60s (RAM only)` | Stillness window in seconds |
| `gps.th <0..255>` | `OK th=20 applied (RAM only)` | ANY_MOT_TH; 7.8125 mg per count at +/-4G |
| `gps.motion` | `th=8 dur=3 bw=0 fsr=2 intpin=4 int=1 mot=7 spur=0` | Register readback plus INT pin level. `int=1` is the idle level (active low); `int=0` means latched |

Anything outside the `gps.*` family is passed back to the stock CLI, so `ver`,
`gps on`, `reboot` and friends behave exactly as before.

`gps.motion` deliberately does not read `INT_ST0` (0x09): with `INT_RD_CLR`
enabled that read clears the latch and would swallow a pending motion event.

## Register configuration

Written by `enableImu()` and cross-checked against three independent sources:
the QST QMA6100P Preliminary Datasheet Rev A1, the official Seeed T1000-E
firmware driver (`qma6100p_motion_init()`), and the RIOT OS / Meshtastic
register headers.

| Register | Value | Purpose |
|---|---|---|
| `0x36` SR | `0xB6`, 5 ms, `0x00`, 10 ms | Soft reset, then release |
| `0x0F` FSR | bits[3:0] = `0b0010` | +/-4G range, 488 ug LSB |
| `0x10` BW | `0x00` | 100 Hz ODR, filters off |
| `0x11` PM | `\|= 0x80` | Active mode |
| `0x18` INT_EN2 | `0x07` | Any-motion enable X, Y, Z |
| `0x2E` MOT_CONF2 | `th` (default 8) | ANY_MOT_TH: 8 x 7.8125 mg = 62.5 mg |
| `0x2C` MOT_CONF0 | `0x03` | 4 consecutive samples (~40 ms) above threshold |
| `0x1A` INT_MAP1 | `\|= 0x01` | INT1_ANY_MOT is **bit 0**, not bit 7 |
| `0x20` INTPIN_CONF | bits[1:0] = `00` | Push-pull, **active low**, matching FALLING + INPUT_PULLUP |
| `0x21` INT_CFG | `\|= 0x81` | Latch interrupt, clear on register read |

Threshold: `ANY_MOT_TH x 16 x range-LSB`, i.e. 7.8125 mg per count at +/-4G.
`th=8` (62.5 mg) is a starting point, not a field-tuned value. Adjust at runtime
with `gps.th`; 0 is close to the noise floor, 255 is about 2 g.

What v1.1 fixed relative to the first draft:

- `0x20` was left at `0x07` (open-drain, **active high**), so the FALLING edge
  never arrived and AUTO could never wake the GPS.
- `0x1A` was `0x80`, which maps NO-motion, not any-motion.
- `0x2E` was never written, leaving the default 0 (trigger on sensor noise).
- A motion event landing between enabling the interrupt and `attachInterrupt()`
  latched the pin low with no edge to follow. The pin is now sampled once after
  attaching and the event is queued manually.

## Flashing

The UF2 targets the stock T1000-E UF2 bootloader. Linker script, bootloader and
radio configuration are unchanged from upstream `companion-v1.18.0`.

1. Back up the firmware you are currently running.
2. Double-press RESET until the `BOOT` drive appears.
3. Drag `firmware.uf2` onto it; the device reboots automatically.
4. Reconnect with the MeshCore App and run `gps.status`.

`ver` reports the upstream string `v1.18.0`; use `gps.status` to confirm this
build is running.

## Building

```sh
pio run -e t1000e_companion_radio_ble
```

Output: `.pio/build/t1000e_companion_radio_ble/firmware.uf2`.

The state machine also has a host-side simulation harness (Arduino/Wire stubs,
I2C fault injection) that runs the real `SmartGps.h`; it lives in the
development workspace outside this repository, so it is not part of the release.

## Known limitations

- AUTO mode, `still` and `th` are RAM only; a reboot returns to the persisted
  App setting.
- Any-motion is a threshold event, not proof of travel. A smooth-moving vehicle
  may not produce repeated interrupts, while a rough surface may produce false
  ones.
- No periodic stationary fix, no manual one-shot fix, no speed fusion.
- `stop_gps()` calls `airohaEnterSleep()`, which can block for up to ~450 ms.
  This was acceptable on hardware but is not free.
- Wake-on-motion from full MCU deep sleep (`sd_app_evt_wait()`) depends on
  GPIOTE behaviour and is the main remaining unknown. Verify on your unit.

## Hardware checklist

| # | Check | Status |
|---|---|---|
| H01 | QMA6100P chip ID, address, power rail | verified |
| H02 | INT1 latch, read-clear, re-trigger; `gps.motion` shows `int=1` at idle | verified |
| H03 | Threshold tuning from `th=8` with `gps.th` | tune per use case |
| H04 | Motion wake to valid NMEA fix (VRTC warm start) | measure |
| H05 | Current draw: ON / AUTO still / AUTO moving | measure |
| H06 | `stop_gps()` blocking effect on BLE/LoRa latency | observe |
| H07 | Motion wake from full MCU deep sleep | verify |
| H08 | BLE/LoRa regression | basic checks pass |
| H09 | Readback of `bw=` / `dur=` on real silicon | verify |

## Credits and licence

MeshCore is MIT licensed (see `license.txt`), copyright Scott Powell /
rippleradios.com. This variant patch is distributed under the same licence and
keeps the original notice.

Register facts come from the QST QMA6100P datasheet and are cross-checked against
the published Seeed T1000-E firmware and the RIOT OS register headers. No
upstream driver code was copied.

This is an unofficial community build. It is not affiliated with or endorsed by
MeshCore or Seeed Studio. Flash at your own risk.

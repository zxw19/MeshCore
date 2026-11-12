#include <Arduino.h>
#include <Wire.h>

#include "T1000eBoard.h"
#include "SmartGps.h"

extern SmartGps smart_gps;   // defined in target.cpp

void T1000eBoard::begin() {
  NRF52BoardDCDC::begin();
  btn_prev_state = HIGH;

#ifdef BUTTON_PIN
  pinMode(BATTERY_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
#endif

#if defined(PIN_BOARD_SDA) && defined(PIN_BOARD_SCL)
  Wire.setPins(PIN_BOARD_SDA, PIN_BOARD_SCL);
#endif

  Wire.begin();

  delay(10);   // give sx1262 some time to power up
}

bool T1000eBoard::handleCommand(const char* command, uint32_t sender_timestamp, char* reply) {
  (void)sender_timestamp;
  return smart_gps.command(command, millis(), reply);
}
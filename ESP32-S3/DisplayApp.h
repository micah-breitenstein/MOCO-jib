#pragma once

#include <Arduino.h>
#include "DisplayBackend.h"

class DisplayApp {
 public:
  void begin();
  void loop();

 private:
  void initializeWavesharePeripherals();
  bool probeI2CDevice(uint8_t address);
  void updateState();

  SerialDisplayBackend backend;
  DisplaySnapshot snapshot;
  unsigned long lastUpdateMs = 0;
  bool touchPresent = false;
  bool imuPresent = false;
};

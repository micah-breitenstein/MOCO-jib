#pragma once

#include <Arduino.h>

struct DisplaySnapshot {
  String mode;
  String status;
  uint16_t frame;
  uint16_t waypoint;
  uint16_t waypointCount;
  uint8_t speedTier;
  uint8_t batteryPercent;
  bool recording;
  bool connected;
  unsigned long uptimeMs;
};

class DisplayBackend {
 public:
  virtual ~DisplayBackend() {}
  virtual void begin() = 0;
  virtual void render(const DisplaySnapshot& snapshot) = 0;
};

class SerialDisplayBackend : public DisplayBackend {
 public:
  void begin() override;
  void render(const DisplaySnapshot& snapshot) override;

 private:
  unsigned long lastPrintMs = 0;
};

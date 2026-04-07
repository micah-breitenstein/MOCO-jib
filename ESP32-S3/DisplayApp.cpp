#include "DisplayApp.h"

#include <Wire.h>

#include "WaveshareBoardConfig.h"

void DisplayApp::begin() {
  Serial.begin(115200);
  delay(50);

  initializeWavesharePeripherals();

  snapshot.mode = "MANUAL";
  snapshot.status = "BOOT";
  snapshot.frame = 0;
  snapshot.waypoint = 1;
  snapshot.waypointCount = 8;
  snapshot.speedTier = 0;
  snapshot.batteryPercent = 100;
  snapshot.recording = false;
  snapshot.connected = touchPresent || imuPresent;
  snapshot.uptimeMs = 0;

  backend.begin();
  backend.render(snapshot);
}

void DisplayApp::initializeWavesharePeripherals() {
  Wire.begin(Waveshare191::kI2cPinSda, Waveshare191::kI2cPinScl, Waveshare191::kI2cClockHz);

  touchPresent = probeI2CDevice(Waveshare191::kTouchAddressFt3168);
  imuPresent = probeI2CDevice(Waveshare191::kImuAddressQmi8658Primary)
            || probeI2CDevice(Waveshare191::kImuAddressQmi8658Secondary);

  Serial.println("Waveshare ESP32-S3 AMOLED 1.91 board profile");
  Serial.print("Display: ");
  Serial.print(Waveshare191::kDisplayWidth);
  Serial.print("x");
  Serial.println(Waveshare191::kDisplayHeight);
  Serial.print("Touch FT3168 @0x38: ");
  Serial.println(touchPresent ? "found" : "not found");
  Serial.print("IMU QMI8658 @0x6A/0x6B: ");
  Serial.println(imuPresent ? "found" : "not found");
}

bool DisplayApp::probeI2CDevice(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void DisplayApp::loop() {
  updateState();
  backend.render(snapshot);
}

void DisplayApp::updateState() {
  const unsigned long now = millis();
  if (now - lastUpdateMs < 100) {
    return;
  }
  lastUpdateMs = now;

  snapshot.uptimeMs = now;
  snapshot.frame = static_cast<uint16_t>((now / 1000UL) % 5000UL);
  snapshot.speedTier = static_cast<uint8_t>((now / 3000UL) % 3UL);
  snapshot.waypoint = static_cast<uint16_t>(((now / 2000UL) % snapshot.waypointCount) + 1);

  if ((now / 8000UL) % 2UL == 0) {
    snapshot.mode = "MANUAL";
    snapshot.status = "READY";
    snapshot.recording = false;
  } else {
    snapshot.mode = "FLOWLAPSE";
    snapshot.status = "CAPTURE";
    snapshot.recording = true;
  }

  if (snapshot.batteryPercent > 10 && (now % 5000UL) < 120UL) {
    snapshot.batteryPercent--;
  }
}

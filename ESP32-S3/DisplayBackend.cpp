#include "DisplayBackend.h"

void SerialDisplayBackend::begin() {
  Serial.println("Display backend: serial");
}

void SerialDisplayBackend::render(const DisplaySnapshot& snapshot) {
  const unsigned long now = millis();
  if (now - lastPrintMs < 250) {
    return;
  }
  lastPrintMs = now;

  Serial.print("[DISPLAY] mode=");
  Serial.print(snapshot.mode);
  Serial.print(" status=");
  Serial.print(snapshot.status);
  Serial.print(" frame=");
  Serial.print(snapshot.frame);
  Serial.print(" waypoint=");
  Serial.print(snapshot.waypoint);
  Serial.print("/");
  Serial.print(snapshot.waypointCount);
  Serial.print(" tier=");
  Serial.print(snapshot.speedTier);
  Serial.print(" battery=");
  Serial.print(snapshot.batteryPercent);
  Serial.print("% rec=");
  Serial.print(snapshot.recording ? "1" : "0");
  Serial.print(" link=");
  Serial.print(snapshot.connected ? "1" : "0");
  Serial.print(" uptimeMs=");
  Serial.println(snapshot.uptimeMs);
}

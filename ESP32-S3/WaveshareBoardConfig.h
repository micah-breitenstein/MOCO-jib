#pragma once

#include <Arduino.h>

namespace Waveshare191 {

constexpr uint16_t kDisplayWidth = 536;
constexpr uint16_t kDisplayHeight = 240;

constexpr int kLcdPinCs = 6;
constexpr int kLcdPinPclk = 47;
constexpr int kLcdPinData0 = 18;
constexpr int kLcdPinData1 = 7;
constexpr int kLcdPinData2 = 48;
constexpr int kLcdPinData3 = 5;
constexpr int kLcdPinReset = 17;

constexpr int kI2cPinSda = 40;
constexpr int kI2cPinScl = 39;
constexpr uint32_t kI2cClockHz = 300000;

constexpr uint8_t kTouchAddressFt3168 = 0x38;
constexpr uint8_t kImuAddressQmi8658Primary = 0x6A;
constexpr uint8_t kImuAddressQmi8658Secondary = 0x6B;

}  // namespace Waveshare191

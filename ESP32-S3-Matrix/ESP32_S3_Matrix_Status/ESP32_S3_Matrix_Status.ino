#include <Adafruit_NeoPixel.h>
#include <math.h>

constexpr uint8_t UART_RX_PIN = 44;
constexpr uint8_t UART_TX_PIN = 43;
constexpr uint32_t UART_BAUD = 115200;

// Set this to the actual onboard RGB matrix data pin for your Waveshare board.
constexpr uint8_t MATRIX_DATA_PIN = 14;
constexpr uint16_t MATRIX_LED_COUNT = 64;

Adafruit_NeoPixel matrixStrip(MATRIX_LED_COUNT, MATRIX_DATA_PIN, NEO_RGB + NEO_KHZ800);

String lineBuffer;
bool errorActive = false;
bool emergencyStopActive = false;
bool droneModeActive = false;
unsigned long lastTwinkleMs = 0;
unsigned long lastStatusRxMs = 0;
unsigned long lastOkPulseMs = 0;
uint8_t lastOkPulseLevel = 0;
uint8_t okPulseDitherAccumulator = 0;
float okPulseSmoothedLevel = 0.0f;

constexpr unsigned long TWINKLE_FAST_PHASE_MS = 3200;
constexpr unsigned long TWINKLE_SLOW_PHASE_MS = 1200;
constexpr unsigned long STRUCTURED_STRIDE_MIN_INTERVAL_MS = 700;
constexpr unsigned long STRUCTURED_STRIDE_MAX_INTERVAL_MS = 2200;
constexpr unsigned long STATUS_SIGNAL_TIMEOUT_MS = 3500;
constexpr unsigned long TWINKLE_UPDATE_FAST_MS = 14;
constexpr unsigned long TWINKLE_UPDATE_SLOW_MS = 28;
constexpr unsigned long TWINKLE_CYCLE_FAST_MS = 360;
constexpr unsigned long TWINKLE_CYCLE_SLOW_MS = 620;
constexpr unsigned long OK_PULSE_CYCLE_MS = 2000;
constexpr unsigned long MODE_INDICATOR_DURATION_MS = 1800;
constexpr uint8_t RANDOM_TWINKLE_CHANCE_PERCENT = 12;
constexpr unsigned long RANDOM_TWINKLE_MIN_MS = 90;
constexpr unsigned long RANDOM_TWINKLE_MAX_MS = 260;
constexpr unsigned long RANDOM_TWINKLE_GAP_MIN_MS = 35;
constexpr unsigned long RANDOM_TWINKLE_GAP_MAX_MS = 420;
constexpr uint8_t BACKGROUND_ROW_SHIMMER_PEAK = 0;
constexpr uint8_t RANDOM_TWINKLE_PEAK_MIN = 70;
constexpr uint8_t RANDOM_TWINKLE_PEAK_MAX = 170;

enum class PulseBrightnessSetting : uint8_t {
  LEVEL_LOW,
  LEVEL_MEDIUM,
  LEVEL_HIGH,
  LEVEL_ULTRA
};

enum class ErrorBrightnessSetting : uint8_t {
  LEVEL_LOW,
  LEVEL_MEDIUM,
  LEVEL_HIGH,
  LEVEL_ULTRA
};

struct PulseBrightnessProfile {
  uint8_t stripBrightness;
  uint8_t pulseMinLevel;
  uint8_t pulseMaxLevel;
  uint8_t blueScale;
};

struct ErrorBrightnessProfile {
  uint8_t stripBrightness;
  uint8_t baseRed;
  uint8_t twinkleWhitePeak;
  uint8_t randomPeakMin;
  uint8_t randomPeakMax;
};

constexpr PulseBrightnessSetting OK_PULSE_BRIGHTNESS_SETTING = PulseBrightnessSetting::LEVEL_LOW;
constexpr ErrorBrightnessSetting ERROR_BRIGHTNESS_SETTING = ErrorBrightnessSetting::LEVEL_LOW;

uint8_t okPulseMinLevel = 60;
uint8_t okPulseMaxLevel = 90;
uint8_t okPulseBlueScale = 255;
uint8_t baseErrorRed = 200;
uint8_t twinkleWhitePeak = 200;
uint8_t randomTwinklePeakMin = 70;
uint8_t randomTwinklePeakMax = 170;
uint8_t errorStripBrightness = 255;

PulseBrightnessProfile getPulseBrightnessProfile(PulseBrightnessSetting setting) {
  switch (setting) {
    case PulseBrightnessSetting::LEVEL_LOW:
      return {5, 68, 82, 255};
    case PulseBrightnessSetting::LEVEL_MEDIUM:
      return {64, 68, 82, 204};
    case PulseBrightnessSetting::LEVEL_HIGH:
      return {128, 68, 82, 204};
    case PulseBrightnessSetting::LEVEL_ULTRA:
      return {255, 68, 82, 204};
  }

  return {5, 68, 82, 255};
}

ErrorBrightnessProfile getErrorBrightnessProfile(ErrorBrightnessSetting setting) {
  switch (setting) {
    case ErrorBrightnessSetting::LEVEL_LOW:
      return {48, 80, 80, 30, 70};
    case ErrorBrightnessSetting::LEVEL_MEDIUM:
      return {96, 130, 130, 45, 105};
    case ErrorBrightnessSetting::LEVEL_HIGH:
      return {160, 170, 170, 60, 140};
    case ErrorBrightnessSetting::LEVEL_ULTRA:
      return {255, 220, 220, 80, 190};
  }

  return {255, 220, 220, 80, 190};
}

unsigned long randomTwinkleStartMs[MATRIX_LED_COUNT] = {0};
unsigned long randomTwinkleEndMs[MATRIX_LED_COUNT] = {0};
unsigned long randomTwinkleNextMs[MATRIX_LED_COUNT] = {0};
uint8_t randomTwinklePeak[MATRIX_LED_COUNT] = {0};
uint8_t structuredStride = 2;
uint8_t structuredStrideIndex = 0;
unsigned long nextStructuredStrideChangeMs = 0;
bool modeIndicatorActive = false;
unsigned long modeIndicatorUntilMs = 0;
uint8_t modeIndicatorR = 0;
uint8_t modeIndicatorG = 0;
uint8_t modeIndicatorB = 0;

void updateStructuredStride(unsigned long nowMs) {
  if (nextStructuredStrideChangeMs == 0) {
    nextStructuredStrideChangeMs = nowMs + (unsigned long)random(STRUCTURED_STRIDE_MIN_INTERVAL_MS, STRUCTURED_STRIDE_MAX_INTERVAL_MS + 1);
    return;
  }

  if (nowMs < nextStructuredStrideChangeMs) {
    return;
  }

  constexpr uint8_t STRIDE_SEQUENCE[] = {2};
  constexpr uint8_t STRIDE_SEQUENCE_COUNT = sizeof(STRIDE_SEQUENCE) / sizeof(STRIDE_SEQUENCE[0]);
  structuredStrideIndex = (uint8_t)((structuredStrideIndex + 1) % STRIDE_SEQUENCE_COUNT);
  structuredStride = STRIDE_SEQUENCE[structuredStrideIndex];
  nextStructuredStrideChangeMs = nowMs + (unsigned long)random(STRUCTURED_STRIDE_MIN_INTERVAL_MS, STRUCTURED_STRIDE_MAX_INTERVAL_MS + 1);
}

bool isStructuredPixel(uint16_t index, unsigned long nowMs) {
  (void)nowMs;
  if (structuredStride <= 1) {
    return true;
  }

  return (index % structuredStride) == 0U;
}

void applyErrorBrightnessSetting() {
  ErrorBrightnessProfile profile = getErrorBrightnessProfile(ERROR_BRIGHTNESS_SETTING);
  errorStripBrightness = profile.stripBrightness;
  baseErrorRed = profile.baseRed;
  twinkleWhitePeak = profile.twinkleWhitePeak;
  randomTwinklePeakMin = profile.randomPeakMin;
  randomTwinklePeakMax = profile.randomPeakMax;
}

void resetRandomTwinkleSchedule(unsigned long nowMs) {
  for (uint16_t i = 0; i < MATRIX_LED_COUNT; ++i) {
    randomTwinkleStartMs[i] = 0;
    randomTwinkleEndMs[i] = 0;
    randomTwinklePeak[i] = 0;

    if (i % 3 == 0) {
      randomTwinkleNextMs[i] = 0;
    } else {
      randomTwinkleNextMs[i] = nowMs + (unsigned long)random(RANDOM_TWINKLE_GAP_MIN_MS, RANDOM_TWINKLE_GAP_MAX_MS + 1);
    }
  }
}

void updateRandomTwinkles(unsigned long nowMs) {
  for (uint16_t i = 0; i < MATRIX_LED_COUNT; ++i) {
    if (isStructuredPixel(i, nowMs)) {
      continue;
    }

    if (nowMs < randomTwinkleNextMs[i]) {
      continue;
    }

    if (random(100) < RANDOM_TWINKLE_CHANCE_PERCENT) {
      randomTwinkleStartMs[i] = nowMs;
      randomTwinkleEndMs[i] = nowMs + (unsigned long)random(RANDOM_TWINKLE_MIN_MS, RANDOM_TWINKLE_MAX_MS + 1);
      randomTwinklePeak[i] = (uint8_t)random(randomTwinklePeakMin, randomTwinklePeakMax + 1);
    }

    randomTwinkleNextMs[i] = nowMs + (unsigned long)random(RANDOM_TWINKLE_GAP_MIN_MS, RANDOM_TWINKLE_GAP_MAX_MS + 1);
  }
}

void renderErrorTwinkleFrame(unsigned long nowMs, unsigned long twinkleCycleMs) {
  const unsigned long halfCycle = twinkleCycleMs / 2;

  for (uint16_t i = 0; i < MATRIX_LED_COUNT; ++i) {
    unsigned long pixelPhase = (nowMs + ((unsigned long)i * 53UL)) % twinkleCycleMs;
    unsigned long pixelRamp = (pixelPhase <= halfCycle) ? pixelPhase : (twinkleCycleMs - pixelPhase);
    uint8_t pixelShimmer = (uint8_t)((pixelRamp * BACKGROUND_ROW_SHIMMER_PEAK) / halfCycle);

    uint16_t baseRedMixed = (uint16_t)baseErrorRed + (pixelShimmer / 2);
    uint8_t r = (baseRedMixed > 255U) ? 255U : (uint8_t)baseRedMixed;
    uint8_t g = pixelShimmer / 12;
    uint8_t b = pixelShimmer / 2;

    if (isStructuredPixel(i, nowMs)) {
      unsigned long phase = (nowMs + (i * 83UL)) % twinkleCycleMs;
      unsigned long ramp = (phase <= halfCycle) ? phase : (twinkleCycleMs - phase);
      uint8_t white = (uint8_t)((ramp * twinkleWhitePeak) / halfCycle);

      uint16_t redMixed = (uint16_t)baseErrorRed + white;
      r = (redMixed > 255U) ? 255U : (uint8_t)redMixed;
      g = white / 10;
      b = white;
    } else if (randomTwinkleEndMs[i] > randomTwinkleStartMs[i] && nowMs < randomTwinkleEndMs[i]) {
      unsigned long duration = randomTwinkleEndMs[i] - randomTwinkleStartMs[i];
      unsigned long elapsed = nowMs - randomTwinkleStartMs[i];
      unsigned long halfDuration = duration / 2;

      if (halfDuration == 0) {
        halfDuration = 1;
      }

      unsigned long ramp = (elapsed <= halfDuration) ? elapsed : (duration - elapsed);
      uint8_t white = (uint8_t)((ramp * randomTwinklePeak[i]) / halfDuration);

      uint16_t redMixed = (uint16_t)baseErrorRed + white;
      r = (redMixed > 255U) ? 255U : (uint8_t)redMixed;
      g = white / 10;
      b = white;
    }

    matrixStrip.setPixelColor(i, matrixStrip.Color(r, g, b));
  }
  matrixStrip.show();
}

void fillMatrix(uint8_t r, uint8_t g, uint8_t b) {
  matrixStrip.fill(matrixStrip.Color(r, g, b));
  matrixStrip.show();
}

void applyPulseBrightnessSetting() {
  PulseBrightnessProfile profile = getPulseBrightnessProfile(OK_PULSE_BRIGHTNESS_SETTING);
  okPulseMinLevel = profile.pulseMinLevel;
  okPulseMaxLevel = profile.pulseMaxLevel;
  okPulseBlueScale = profile.blueScale;
  matrixStrip.setBrightness(profile.stripBrightness);
}

void renderOkPulseFrame(uint8_t pulseBase, uint8_t frac255) {
  uint8_t distributedLevel = pulseBase;
  uint8_t distributedBlue = (uint8_t)(((uint16_t)pulseBase * okPulseBlueScale) / 255U);

  for (uint16_t i = 0; i < MATRIX_LED_COUNT; ++i) {
    uint16_t orderedIndex = i;
    uint32_t threshold = ((uint32_t)orderedIndex * 255U) / MATRIX_LED_COUNT;

    if (pulseBase < okPulseMaxLevel && threshold < frac255) {
      distributedLevel = pulseBase + 1;
      distributedBlue = (uint8_t)(((uint16_t)distributedLevel * okPulseBlueScale) / 255U);
    } else {
      distributedLevel = pulseBase;
      distributedBlue = (uint8_t)(((uint16_t)pulseBase * okPulseBlueScale) / 255U);
    }

    matrixStrip.setPixelColor(i, matrixStrip.Color(distributedLevel, distributedLevel, distributedBlue));
  }

  matrixStrip.show();
}

void showError() {
  emergencyStopActive = false;
  droneModeActive = false;
  errorActive = true;
  lastStatusRxMs = millis();
  applyErrorBrightnessSetting();
  matrixStrip.setBrightness(errorStripBrightness);
  structuredStride = 2;
  structuredStrideIndex = 0;
  nextStructuredStrideChangeMs = 0;
  resetRandomTwinkleSchedule(lastStatusRxMs);
  renderErrorTwinkleFrame(millis(), TWINKLE_CYCLE_FAST_MS);
}

void showOk() {
  emergencyStopActive = false;
  droneModeActive = false;
  errorActive = false;
  modeIndicatorActive = false;
  lastStatusRxMs = millis();
  applyPulseBrightnessSetting();
  lastOkPulseMs = lastStatusRxMs;
  lastOkPulseLevel = 0;
  okPulseDitherAccumulator = 0;
  okPulseSmoothedLevel = okPulseMinLevel;
  fillMatrix(okPulseMinLevel, okPulseMinLevel, (uint8_t)(((uint16_t)okPulseMinLevel * okPulseBlueScale) / 255U));
}

void showModeIndicator(const String& modeText) {
  uint8_t r = 100;
  uint8_t g = 100;
  uint8_t b = 100;

  if (modeText.startsWith("DRONE")) {
    r = 130;
    g = 20;
    b = 170;
  } else if (modeText.startsWith("TIMELAPSE")) {
    r = 170;
    g = 120;
    b = 0;
  } else if (modeText.startsWith("BOUNCE")) {
    r = 0;
    g = 130;
    b = 170;
  } else if (modeText.startsWith("MANUAL")) {
    r = 120;
    g = 120;
    b = 120;
  }

  errorActive = false;
  applyPulseBrightnessSetting();
  modeIndicatorActive = true;
  modeIndicatorUntilMs = millis() + MODE_INDICATOR_DURATION_MS;
  modeIndicatorR = r;
  modeIndicatorG = g;
  modeIndicatorB = b;
  fillMatrix(modeIndicatorR, modeIndicatorG, modeIndicatorB);
}

void showEmergencyStopActive() {
  emergencyStopActive = true;
  errorActive = false;
  modeIndicatorActive = false;
  applyErrorBrightnessSetting();
  matrixStrip.setBrightness(errorStripBrightness);
  structuredStride = 2;
  structuredStrideIndex = 0;
  nextStructuredStrideChangeMs = 0;
  unsigned long nowMs = millis();
  resetRandomTwinkleSchedule(nowMs);
  lastTwinkleMs = nowMs;
  renderErrorTwinkleFrame(nowMs, TWINKLE_CYCLE_FAST_MS);
}

void showEmergencyStopReleased() {
  emergencyStopActive = false;
  if (droneModeActive) {
    showDroneModeHold();
  } else {
    showOk();
  }
}

void showDroneModeHold() {
  emergencyStopActive = false;
  errorActive = false;
  modeIndicatorActive = false;
  droneModeActive = true;
  applyPulseBrightnessSetting();
  fillMatrix(130, 20, 170);
}

void handleStatusLine(const String& line) {
  if (line.startsWith("EMERGENCY_STOP:ACTIVE")
      || (line.startsWith("EMERGENCY STOP") && line.indexOf("RELEASED") < 0)) {
    showEmergencyStopActive();
    Serial.println("Matrix status: EMERGENCY STOP -> FLASH RED");
    return;
  }

  if (line.startsWith("EMERGENCY_STOP:RELEASED") || line.startsWith("EMERGENCY STOP RELEASED")) {
    showEmergencyStopReleased();
    Serial.println("Matrix status: EMERGENCY STOP RELEASED -> OK");
    return;
  }

  if (emergencyStopActive) {
    Serial.println("Matrix status: emergency latched, ignoring non-release status line");
    return;
  }

  if (line.startsWith("CONTROLLER_ERROR:")) {
    showError();
    Serial.println("Matrix status: ERROR -> RED");
    return;
  }

  if (line.startsWith("CONTROLLER_OK:")) {
    if (!droneModeActive) {
      showOk();
      Serial.println("Matrix status: OK -> OFF");
    }
    return;
  }

  if (line.startsWith("MODE:")) {
    String modeMsg = line.substring(5);
    modeMsg.trim();
    if (modeMsg.startsWith("DRONE")) {
      showDroneModeHold();
      Serial.println("Matrix mode indicator: DRONE (latched pink)");
    } else {
      bool wasDroneMode = droneModeActive;
      droneModeActive = false;
      if (wasDroneMode) {
        showOk();
        Serial.print("Matrix mode transition: DRONE -> ");
        Serial.print(modeMsg);
        Serial.println(" (direct to dim OK)");
      } else {
        showModeIndicator(modeMsg);
        Serial.print("Matrix mode indicator: ");
        Serial.println(modeMsg);
      }
    }
    return;
  }

  Serial.print("Matrix received unhandled line: ");
  Serial.println(line);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  randomSeed((uint32_t)(micros() ^ millis()));

  matrixStrip.begin();
  applyPulseBrightnessSetting();
  applyErrorBrightnessSetting();
  fillMatrix(50, 50, 50);
  delay(200);
  showOk();
  lastStatusRxMs = millis();

  Serial.println("ESP32-S3 Matrix status listener started");
  Serial.print("UART1 RX=");
  Serial.print(UART_RX_PIN);
  Serial.print(" TX=");
  Serial.println(UART_TX_PIN);
}

void loop() {
  unsigned long now = millis();

  if (!emergencyStopActive && errorActive && (now - lastStatusRxMs > STATUS_SIGNAL_TIMEOUT_MS)) {
    showOk();
    Serial.println("Matrix status timeout -> OK (dim green)");
  }

  if (emergencyStopActive) {
    const unsigned long phaseSpan = TWINKLE_FAST_PHASE_MS + TWINKLE_SLOW_PHASE_MS;
    const unsigned long phasePos = now % phaseSpan;
    bool fastPhase = phasePos < TWINKLE_FAST_PHASE_MS;
    unsigned long updateInterval = fastPhase ? TWINKLE_UPDATE_FAST_MS : TWINKLE_UPDATE_SLOW_MS;
    unsigned long twinkleCycle = fastPhase ? TWINKLE_CYCLE_FAST_MS : TWINKLE_CYCLE_SLOW_MS;

    updateStructuredStride(now);
    updateRandomTwinkles(now);

    if (now - lastTwinkleMs >= updateInterval) {
      lastTwinkleMs = now;
      renderErrorTwinkleFrame(now, twinkleCycle);
    }
  } else if (droneModeActive) {
    fillMatrix(130, 20, 170);
  } else if (errorActive) {
    updateStructuredStride(now);
    updateRandomTwinkles(now);

    const unsigned long phaseSpan = TWINKLE_FAST_PHASE_MS + TWINKLE_SLOW_PHASE_MS;
    const unsigned long phasePos = now % phaseSpan;
    bool fastPhase = phasePos < TWINKLE_FAST_PHASE_MS;
    unsigned long updateInterval = fastPhase ? TWINKLE_UPDATE_FAST_MS : TWINKLE_UPDATE_SLOW_MS;
    unsigned long twinkleCycle = fastPhase ? TWINKLE_CYCLE_FAST_MS : TWINKLE_CYCLE_SLOW_MS;

    if (now - lastTwinkleMs >= updateInterval) {
      lastTwinkleMs = now;
      renderErrorTwinkleFrame(now, twinkleCycle);
    }
  } else if (modeIndicatorActive) {
    if (now >= modeIndicatorUntilMs) {
      showOk();
    } else {
      fillMatrix(modeIndicatorR, modeIndicatorG, modeIndicatorB);
    }
  } else {
    unsigned long phase = now % OK_PULSE_CYCLE_MS;
    float phaseRatio = (float)phase / (float)OK_PULSE_CYCLE_MS;
    float eased = 0.5f - 0.5f * cosf(phaseRatio * 6.28318530718f);
    float pulseFloat = okPulseMinLevel + eased * (float)(okPulseMaxLevel - okPulseMinLevel);
    if (pulseFloat > okPulseSmoothedLevel) {
      okPulseSmoothedLevel += (pulseFloat - okPulseSmoothedLevel) * 0.35f;
    } else {
      okPulseSmoothedLevel += (pulseFloat - okPulseSmoothedLevel) * 0.18f;
    }

    uint8_t pulseBase = (uint8_t)okPulseSmoothedLevel;
    uint8_t pulseLevel = pulseBase;
    uint8_t frac255 = (uint8_t)((okPulseSmoothedLevel - (float)pulseBase) * 255.0f);

    okPulseDitherAccumulator = (uint8_t)(okPulseDitherAccumulator + frac255);
    if (okPulseDitherAccumulator < frac255 && pulseLevel < okPulseMaxLevel) {
      pulseLevel++;
    }

    if (now - lastOkPulseMs >= 5) {
      lastOkPulseMs = now;
      lastOkPulseLevel = pulseLevel;
      renderOkPulseFrame(pulseBase, frac255);
    }
  }

  while (Serial1.available() > 0) {
    char c = static_cast<char>(Serial1.read());

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      lineBuffer.trim();
      if (lineBuffer.length() > 0) {
        handleStatusLine(lineBuffer);
      }
      lineBuffer = "";
      continue;
    }

    if (lineBuffer.length() < 255) {
      lineBuffer += c;
    } else {
      lineBuffer = "";
    }
  }
}

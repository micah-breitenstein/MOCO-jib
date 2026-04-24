#include <Adafruit_NeoPixel.h>
#include <Preferences.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// =========================
// Hardware / Transport
// =========================

constexpr uint8_t UART_RX_PIN = 44;
constexpr uint8_t UART_TX_PIN = 43;
constexpr uint32_t UART_BAUD = 115200;

constexpr uint8_t MATRIX_DATA_PIN = 14;
constexpr uint16_t MATRIX_LED_COUNT = 64;

constexpr size_t UART_LINE_BUFFER_SIZE = 256;

Adafruit_NeoPixel matrixStrip(MATRIX_LED_COUNT, MATRIX_DATA_PIN, NEO_RGB + NEO_KHZ800);
Preferences prefs;

// =========================
// Timing / Visual constants
// =========================

constexpr unsigned long TWINKLE_FAST_PHASE_MS = 3200;
constexpr unsigned long TWINKLE_SLOW_PHASE_MS = 1200;
constexpr unsigned long STRUCTURED_STRIDE_MIN_INTERVAL_MS = 700;
constexpr unsigned long STRUCTURED_STRIDE_MAX_INTERVAL_MS = 2200;
constexpr unsigned long STATUS_SIGNAL_TIMEOUT_MS = 3500;
constexpr unsigned long TWINKLE_UPDATE_FAST_MS = 14;
constexpr unsigned long TWINKLE_UPDATE_SLOW_MS = 28;
constexpr unsigned long TWINKLE_CYCLE_FAST_MS = 360;
constexpr unsigned long TWINKLE_CYCLE_SLOW_MS = 620;
constexpr unsigned long HOME_FLASH_DURATION_MS = 350;
constexpr unsigned long HOME_RETURN_PULSE_CYCLE_MS = 1800;
constexpr unsigned long IDLE_BREATHING_CYCLE_MS = 3600;

constexpr uint8_t RANDOM_TWINKLE_CHANCE_PERCENT = 12;
constexpr unsigned long RANDOM_TWINKLE_MIN_MS = 90;
constexpr unsigned long RANDOM_TWINKLE_MAX_MS = 260;
constexpr unsigned long RANDOM_TWINKLE_GAP_MIN_MS = 35;
constexpr unsigned long RANDOM_TWINKLE_GAP_MAX_MS = 420;
constexpr uint8_t BACKGROUND_ROW_SHIMMER_PEAK = 0;
constexpr uint8_t OK_IDLE_WHITE_LEVEL = 64;

constexpr char PREF_NS[] = "matrix_cfg";
constexpr char PREF_KEY_BRIGHT[] = "mtx_brt";

// =========================
// State Layer
// =========================

enum class Mode : uint8_t {
  IDLE,
  DRONE,
  TIMELAPSE,
  BOUNCE
};

enum class UiMode : uint8_t {
  NORMAL,
  SETTINGS,
  EDIT,
  CONFIRM
};

enum class EventType {
  CONTROLLER_OK,
  CONTROLLER_ERROR,
  MODE_CHANGE,
  EMERGENCY_STOP_ACTIVE,
  EMERGENCY_STOP_RELEASED,
  SETTINGS_OPEN,
  SETTINGS_CLOSE,
  SET_BRIGHTNESS,
  HOME_SET,
  HOME_RETURN,
  HOME_COMPLETE,
  HOME_NOT_SET
};

enum class FlashCue : uint8_t {
  NONE,
  HOME_SET,
  HOME_COMPLETE,
  ERROR_BLIP,
  MODE_CHANGE,
  HOME_NOT_SET
};

struct SystemState {
  bool errorActive;
  bool emergencyStopActive;
  UiMode uiMode;
  Mode currentMode;
  uint8_t brightness;  // 0..100
  bool homeSet;
  bool returningHome;

  // Additional state used by update/render flow.
  unsigned long lastStatusRxMs;
  FlashCue flashCue;
  unsigned long flashCueUntilMs;
};

struct SettingsSnapshot {
  bool errorActive;
  Mode currentMode;
  bool homeSet;
  bool returningHome;
};

SystemState systemState = {
  false,
  false,
  UiMode::NORMAL,
  Mode::IDLE,
  5,
  false,
  false,
  0,
  FlashCue::NONE,
  0
};

SettingsSnapshot settingsSnapshot = {false, Mode::IDLE, false, false};

// =========================
// Render Layer data
// =========================

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

struct RenderContext {
  unsigned long lastTwinkleMs;
  uint8_t structuredStride;
  uint8_t structuredStrideIndex;
  unsigned long nextStructuredStrideChangeMs;
  unsigned long randomTwinkleStartMs[MATRIX_LED_COUNT];
  unsigned long randomTwinkleEndMs[MATRIX_LED_COUNT];
  unsigned long randomTwinkleNextMs[MATRIX_LED_COUNT];
  uint8_t randomTwinklePeak[MATRIX_LED_COUNT];
};

RenderContext renderContext = {
  0,
  2,
  0,
  0,
  {0},
  {0},
  {0},
  {0}
};

// =========================
// Input Layer buffers
// =========================

char uartLineBuffer[UART_LINE_BUFFER_SIZE] = {0};
size_t uartLineLen = 0;

// =========================
// Utility helpers
// =========================

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

bool startsWith(const char* text, const char* prefix) {
  return strncmp(text, prefix, strlen(prefix)) == 0;
}

bool matchesAny(const char* line, const char* a, const char* b, const char* c = nullptr, const char* d = nullptr) {
  if (startsWith(line, a) || startsWith(line, b)) {
    return true;
  }
  if (c != nullptr && startsWith(line, c)) {
    return true;
  }
  if (d != nullptr && startsWith(line, d)) {
    return true;
  }
  return false;
}

void trimLineInPlace(char* line) {
  size_t len = strlen(line);
  while (len > 0 && isspace(static_cast<unsigned char>(line[len - 1])) != 0) {
    line[--len] = '\0';
  }

  size_t start = 0;
  while (line[start] != '\0' && isspace(static_cast<unsigned char>(line[start])) != 0) {
    start++;
  }

  if (start > 0) {
    memmove(line, line + start, strlen(line + start) + 1);
  }
}

void fillMatrix(uint8_t r, uint8_t g, uint8_t b) {
  matrixStrip.fill(matrixStrip.Color(r, g, b));
  matrixStrip.show();
}

void updateStructuredStride(unsigned long nowMs) {
  if (renderContext.nextStructuredStrideChangeMs == 0) {
    renderContext.nextStructuredStrideChangeMs =
      nowMs + static_cast<unsigned long>(random(STRUCTURED_STRIDE_MIN_INTERVAL_MS, STRUCTURED_STRIDE_MAX_INTERVAL_MS + 1));
    return;
  }

  if (nowMs < renderContext.nextStructuredStrideChangeMs) {
    return;
  }

  constexpr uint8_t STRIDE_SEQUENCE[] = {2};
  constexpr uint8_t STRIDE_SEQUENCE_COUNT = sizeof(STRIDE_SEQUENCE) / sizeof(STRIDE_SEQUENCE[0]);

  renderContext.structuredStrideIndex =
    static_cast<uint8_t>((renderContext.structuredStrideIndex + 1) % STRIDE_SEQUENCE_COUNT);
  renderContext.structuredStride = STRIDE_SEQUENCE[renderContext.structuredStrideIndex];
  renderContext.nextStructuredStrideChangeMs =
    nowMs + static_cast<unsigned long>(random(STRUCTURED_STRIDE_MIN_INTERVAL_MS, STRUCTURED_STRIDE_MAX_INTERVAL_MS + 1));
}

bool isStructuredPixel(uint16_t index) {
  if (renderContext.structuredStride <= 1) {
    return true;
  }
  return (index % renderContext.structuredStride) == 0U;
}

void resetRandomTwinkleSchedule(unsigned long nowMs, uint8_t randomPeakMin, uint8_t randomPeakMax) {
  for (uint16_t i = 0; i < MATRIX_LED_COUNT; ++i) {
    renderContext.randomTwinkleStartMs[i] = 0;
    renderContext.randomTwinkleEndMs[i] = 0;
    renderContext.randomTwinklePeak[i] = 0;

    if (i % 3 == 0) {
      renderContext.randomTwinkleNextMs[i] = 0;
    } else {
      renderContext.randomTwinkleNextMs[i] =
        nowMs + static_cast<unsigned long>(random(RANDOM_TWINKLE_GAP_MIN_MS, RANDOM_TWINKLE_GAP_MAX_MS + 1));
    }
  }

  (void)randomPeakMin;
  (void)randomPeakMax;
}

void updateRandomTwinkles(unsigned long nowMs, uint8_t randomPeakMin, uint8_t randomPeakMax) {
  for (uint16_t i = 0; i < MATRIX_LED_COUNT; ++i) {
    if (isStructuredPixel(i)) {
      continue;
    }

    if (nowMs < renderContext.randomTwinkleNextMs[i]) {
      continue;
    }

    if (random(100) < RANDOM_TWINKLE_CHANCE_PERCENT) {
      renderContext.randomTwinkleStartMs[i] = nowMs;
      renderContext.randomTwinkleEndMs[i] =
        nowMs + static_cast<unsigned long>(random(RANDOM_TWINKLE_MIN_MS, RANDOM_TWINKLE_MAX_MS + 1));
      renderContext.randomTwinklePeak[i] = static_cast<uint8_t>(random(randomPeakMin, randomPeakMax + 1));
    }

    renderContext.randomTwinkleNextMs[i] =
      nowMs + static_cast<unsigned long>(random(RANDOM_TWINKLE_GAP_MIN_MS, RANDOM_TWINKLE_GAP_MAX_MS + 1));
  }
}

void renderErrorTwinkleFrame(unsigned long nowMs,
                             unsigned long twinkleCycleMs,
                             uint8_t baseErrorRed,
                             uint8_t twinkleWhitePeak,
                             uint8_t randomPeakMin,
                             uint8_t randomPeakMax) {
  const unsigned long halfCycle = twinkleCycleMs / 2;

  for (uint16_t i = 0; i < MATRIX_LED_COUNT; ++i) {
    unsigned long pixelPhase = (nowMs + (static_cast<unsigned long>(i) * 53UL)) % twinkleCycleMs;
    unsigned long pixelRamp = (pixelPhase <= halfCycle) ? pixelPhase : (twinkleCycleMs - pixelPhase);
    uint8_t pixelShimmer = static_cast<uint8_t>((pixelRamp * BACKGROUND_ROW_SHIMMER_PEAK) / halfCycle);

    uint16_t baseRedMixed = static_cast<uint16_t>(baseErrorRed) + (pixelShimmer / 2);
    uint8_t r = (baseRedMixed > 255U) ? 255U : static_cast<uint8_t>(baseRedMixed);
    uint8_t g = pixelShimmer / 12;
    uint8_t b = pixelShimmer / 2;

    if (isStructuredPixel(i)) {
      unsigned long phase = (nowMs + (static_cast<unsigned long>(i) * 83UL)) % twinkleCycleMs;
      unsigned long ramp = (phase <= halfCycle) ? phase : (twinkleCycleMs - phase);
      uint8_t white = static_cast<uint8_t>((ramp * twinkleWhitePeak) / halfCycle);

      uint16_t redMixed = static_cast<uint16_t>(baseErrorRed) + white;
      r = (redMixed > 255U) ? 255U : static_cast<uint8_t>(redMixed);
      g = white / 10;
      b = white;
    } else if (renderContext.randomTwinkleEndMs[i] > renderContext.randomTwinkleStartMs[i]
               && nowMs < renderContext.randomTwinkleEndMs[i]) {
      unsigned long duration = renderContext.randomTwinkleEndMs[i] - renderContext.randomTwinkleStartMs[i];
      unsigned long elapsed = nowMs - renderContext.randomTwinkleStartMs[i];
      unsigned long halfDuration = duration / 2;
      if (halfDuration == 0) {
        halfDuration = 1;
      }

      unsigned long ramp = (elapsed <= halfDuration) ? elapsed : (duration - elapsed);
      uint8_t white = static_cast<uint8_t>((ramp * renderContext.randomTwinklePeak[i]) / halfDuration);

      uint16_t redMixed = static_cast<uint16_t>(baseErrorRed) + white;
      r = (redMixed > 255U) ? 255U : static_cast<uint8_t>(redMixed);
      g = white / 10;
      b = white;
    }

    matrixStrip.setPixelColor(i, matrixStrip.Color(r, g, b));
  }

  matrixStrip.show();

  (void)randomPeakMin;
  (void)randomPeakMax;
}

uint8_t scalePercentToBrightness(uint8_t percent) {
  if (percent > 100U) {
    percent = 100U;
  }
  return static_cast<uint8_t>((255U * static_cast<uint16_t>(percent)) / 100U);
}

// =========================
// State update / events
// =========================

void queueFlashCue(SystemState& state, FlashCue cue, unsigned long durationMs) {
  state.flashCue = cue;
  state.flashCueUntilMs = millis() + durationMs;
}

Mode parseModeFromPayload(const char* payload) {
  if (payload == nullptr) {
    return Mode::IDLE;
  }

  if (startsWith(payload, "DRONE")) {
    return Mode::DRONE;
  }
  if (startsWith(payload, "TIMELAPSE")) {
    return Mode::TIMELAPSE;
  }
  if (startsWith(payload, "BOUNCE")) {
    return Mode::BOUNCE;
  }
  return Mode::IDLE;
}

void handleEvent(EventType type, const char* payload) {
  const unsigned long now = millis();

  if (type != EventType::EMERGENCY_STOP_RELEASED && systemState.emergencyStopActive) {
    Serial.println("Matrix status: emergency latched, ignoring non-release event");
    return;
  }

  if (systemState.uiMode == UiMode::SETTINGS) {
    if (type == EventType::CONTROLLER_ERROR) {
      settingsSnapshot.errorActive = true;
      return;
    }
    if (type == EventType::CONTROLLER_OK) {
      settingsSnapshot.errorActive = false;
      return;
    }
    if (type == EventType::MODE_CHANGE) {
      settingsSnapshot.currentMode = parseModeFromPayload(payload);
      return;
    }
    if (type != EventType::SETTINGS_CLOSE && type != EventType::SET_BRIGHTNESS) {
      return;
    }
  }

  switch (type) {
    case EventType::CONTROLLER_ERROR:
      systemState.errorActive = true;
      systemState.emergencyStopActive = false;
      systemState.lastStatusRxMs = now;
      resetRandomTwinkleSchedule(now, 70, 170);
      Serial.println("Matrix status: ERROR -> RED");
      break;

    case EventType::CONTROLLER_OK:
      systemState.errorActive = false;
      systemState.emergencyStopActive = false;
      systemState.lastStatusRxMs = now;
      Serial.println("Matrix status: OK");
      break;

    case EventType::MODE_CHANGE:
      systemState.errorActive = false;
      systemState.currentMode = parseModeFromPayload(payload);
      if (systemState.currentMode == Mode::DRONE) {
        Serial.println("Matrix mode: DRONE");
      } else if (systemState.currentMode == Mode::TIMELAPSE) {
        Serial.println("Matrix mode: TIMELAPSE");
      } else if (systemState.currentMode == Mode::BOUNCE) {
        Serial.println("Matrix mode: BOUNCE");
      } else {
        Serial.println("Matrix mode: IDLE");
      }
      break;

    case EventType::EMERGENCY_STOP_ACTIVE:
      systemState.emergencyStopActive = true;
      systemState.errorActive = false;
      resetRandomTwinkleSchedule(now, 70, 170);
      renderContext.lastTwinkleMs = now;
      Serial.println("Matrix status: EMERGENCY STOP -> FLASH RED");
      break;

    case EventType::EMERGENCY_STOP_RELEASED:
      systemState.emergencyStopActive = false;
      Serial.println("Matrix status: EMERGENCY STOP RELEASED");
      break;

    case EventType::SETTINGS_OPEN:
      settingsSnapshot.errorActive = systemState.errorActive;
      settingsSnapshot.currentMode = systemState.currentMode;
      settingsSnapshot.homeSet = systemState.homeSet;
      settingsSnapshot.returningHome = systemState.returningHome;

      systemState.uiMode = UiMode::SETTINGS;
      systemState.errorActive = false;
      systemState.emergencyStopActive = false;
      Serial.println("Matrix status: SETTINGS OPEN -> ORANGE");
      break;

    case EventType::SETTINGS_CLOSE:
      systemState.uiMode = UiMode::NORMAL;
      systemState.errorActive = settingsSnapshot.errorActive;
      systemState.currentMode = settingsSnapshot.currentMode;
      systemState.homeSet = settingsSnapshot.homeSet;
      systemState.returningHome = settingsSnapshot.returningHome;
      Serial.println("Matrix status: SETTINGS CLOSE -> restored");
      break;

    case EventType::SET_BRIGHTNESS: {
      if (payload == nullptr) {
        break;
      }
      int val = atoi(payload);
      if (val >= 0 && val <= 100) {
        systemState.brightness = static_cast<uint8_t>(val);
        prefs.putUChar(PREF_KEY_BRIGHT, systemState.brightness);
        Serial.print("Matrix brightness set to ");
        Serial.print(systemState.brightness);
        Serial.println("%");
      }
      break;
    }

    case EventType::HOME_SET:
      systemState.homeSet = true;
      queueFlashCue(systemState, FlashCue::HOME_SET, HOME_FLASH_DURATION_MS);
      Serial.println("Matrix home: SET -> green flash");
      break;

    case EventType::HOME_RETURN:
      systemState.flashCue = FlashCue::NONE;
      if (systemState.homeSet) {
        systemState.returningHome = true;
      }
      Serial.println("Matrix home: RETURNING -> blue pulse");
      break;

    case EventType::HOME_COMPLETE:
      systemState.returningHome = false;
      queueFlashCue(systemState, FlashCue::HOME_COMPLETE, HOME_FLASH_DURATION_MS);
      Serial.println("Matrix home: COMPLETE -> white flash");
      break;

    case EventType::HOME_NOT_SET:
      systemState.homeSet = false;
      systemState.returningHome = false;
      queueFlashCue(systemState, FlashCue::HOME_NOT_SET, HOME_FLASH_DURATION_MS);
      Serial.println("Matrix home: NOT_SET -> amber flash");
      break;
  }
}

void updateState(SystemState& state, unsigned long now) {
  if (!state.emergencyStopActive && state.uiMode != UiMode::SETTINGS && state.errorActive
      && (now - state.lastStatusRxMs > STATUS_SIGNAL_TIMEOUT_MS)) {
    state.errorActive = false;
    Serial.println("Matrix status timeout -> OK");
  }

  if (state.flashCue != FlashCue::NONE && now >= state.flashCueUntilMs) {
    state.flashCue = FlashCue::NONE;
  }
}

// =========================
// Input Layer
// =========================

bool parseLineToEvent(const char* line, EventType& outType, char* outPayload, size_t outPayloadSize) {
  if (line == nullptr || line[0] == '\0') {
    return false;
  }

  outPayload[0] = '\0';

  if (startsWith(line, "EMERGENCY_STOP:ACTIVE")
      || (startsWith(line, "EMERGENCY STOP") && strstr(line, "RELEASED") == nullptr)) {
    outType = EventType::EMERGENCY_STOP_ACTIVE;
    return true;
  }

  if (startsWith(line, "EMERGENCY_STOP:RELEASED") || startsWith(line, "EMERGENCY STOP RELEASED")) {
    outType = EventType::EMERGENCY_STOP_RELEASED;
    return true;
  }

  if (startsWith(line, "CONTROLLER_ERROR:")) {
    outType = EventType::CONTROLLER_ERROR;
    return true;
  }

  if (startsWith(line, "CONTROLLER_OK:")) {
    outType = EventType::CONTROLLER_OK;
    return true;
  }

  if (startsWith(line, "MODE:")) {
    outType = EventType::MODE_CHANGE;
    strncpy(outPayload, line + 5, outPayloadSize - 1);
    outPayload[outPayloadSize - 1] = '\0';
    trimLineInPlace(outPayload);
    return true;
  }

  if (startsWith(line, "SET:MTX_BRT:")) {
    outType = EventType::SET_BRIGHTNESS;
    strncpy(outPayload, line + 12, outPayloadSize - 1);
    outPayload[outPayloadSize - 1] = '\0';
    trimLineInPlace(outPayload);
    return true;
  }

  if (startsWith(line, "SETTINGS:OPEN")) {
    outType = EventType::SETTINGS_OPEN;
    return true;
  }

  if (startsWith(line, "SETTINGS:CLOSE")) {
    outType = EventType::SETTINGS_CLOSE;
    return true;
  }

  // Home / Zero event aliases for compatibility.
  if (matchesAny(line, "HOME_SET", "HOME:SET")) {
    outType = EventType::HOME_SET;
    return true;
  }
  if (matchesAny(line, "HOME_RETURN", "HOME:RETURN", "HOME:RETURNING")) {
    outType = EventType::HOME_RETURN;
    return true;
  }
  if (matchesAny(line, "HOME_COMPLETE", "HOME:COMPLETE")) {
    outType = EventType::HOME_COMPLETE;
    return true;
  }

  if (matchesAny(line, "HOME_NOT_SET", "HOME:NOT_SET")) {
    outType = EventType::HOME_NOT_SET;
    return true;
  }

  return false;
}

void processInputLine(const char* line) {
  EventType eventType = EventType::CONTROLLER_OK;
  char payload[64] = {0};

  if (parseLineToEvent(line, eventType, payload, sizeof(payload))) {
    handleEvent(eventType, payload[0] == '\0' ? nullptr : payload);
    return;
  }

  Serial.print("Matrix received unhandled line: ");
  Serial.println(line);
}

void processUartInput() {
  while (Serial1.available() > 0) {
    char c = static_cast<char>(Serial1.read());

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      uartLineBuffer[uartLineLen] = '\0';
      trimLineInPlace(uartLineBuffer);
      if (uartLineBuffer[0] != '\0') {
        processInputLine(uartLineBuffer);
      }
      uartLineLen = 0;
      uartLineBuffer[0] = '\0';
      continue;
    }

    if (uartLineLen < UART_LINE_BUFFER_SIZE - 1) {
      uartLineBuffer[uartLineLen++] = c;
    } else {
      // Overflow guard: reset line to avoid partial command processing.
      uartLineLen = 0;
      uartLineBuffer[0] = '\0';
    }
  }
}

// =========================
// Render Layer
// =========================

void renderTwinkleErrorFrame(unsigned long now, const ErrorBrightnessProfile& profile) {
  const unsigned long phaseSpan = TWINKLE_FAST_PHASE_MS + TWINKLE_SLOW_PHASE_MS;
  const unsigned long phasePos = now % phaseSpan;
  bool fastPhase = phasePos < TWINKLE_FAST_PHASE_MS;
  unsigned long updateInterval = fastPhase ? TWINKLE_UPDATE_FAST_MS : TWINKLE_UPDATE_SLOW_MS;
  unsigned long twinkleCycle = fastPhase ? TWINKLE_CYCLE_FAST_MS : TWINKLE_CYCLE_SLOW_MS;

  updateStructuredStride(now);
  updateRandomTwinkles(now, profile.randomPeakMin, profile.randomPeakMax);

  if (now - renderContext.lastTwinkleMs >= updateInterval) {
    renderContext.lastTwinkleMs = now;
    renderErrorTwinkleFrame(now,
                            twinkleCycle,
                            profile.baseRed,
                            profile.twinkleWhitePeak,
                            profile.randomPeakMin,
                            profile.randomPeakMax);
  }
}

bool renderFlashCue(FlashCue cue) {
  switch (cue) {
    case FlashCue::HOME_SET:
      fillMatrix(0, 170, 0);
      return true;
    case FlashCue::HOME_COMPLETE:
      fillMatrix(220, 220, 220);
      return true;
    case FlashCue::HOME_NOT_SET:
      fillMatrix(160, 90, 0);
      return true;
    default:
      return false;
  }
}

void renderCurrentState(const SystemState& state) {
  const PulseBrightnessProfile pulseProfile = getPulseBrightnessProfile(OK_PULSE_BRIGHTNESS_SETTING);
  const ErrorBrightnessProfile errorProfile = getErrorBrightnessProfile(ERROR_BRIGHTNESS_SETTING);
  const unsigned long now = millis();
  matrixStrip.setBrightness(scalePercentToBrightness(state.brightness));

  if (state.emergencyStopActive) {
    renderTwinkleErrorFrame(now, errorProfile);
    return;
  }

  if (renderFlashCue(state.flashCue)) {
    return;
  }

  if (state.uiMode == UiMode::SETTINGS) {
    fillMatrix(255, 50, 0);
    return;
  }

  if (state.returningHome) {
    float phase = static_cast<float>(now % HOME_RETURN_PULSE_CYCLE_MS) / static_cast<float>(HOME_RETURN_PULSE_CYCLE_MS);
    float wave = (sinf(phase * 2.0f * PI) + 1.0f) * 0.5f;
    uint8_t level = static_cast<uint8_t>(20.0f + wave * 130.0f);
    fillMatrix(0, 0, level);
    return;
  }

  if (state.errorActive) {
    renderTwinkleErrorFrame(now, errorProfile);
    return;
  }

  switch (state.currentMode) {
    case Mode::DRONE:
      fillMatrix(130, 20, 170);
      return;
    case Mode::TIMELAPSE:
      fillMatrix(170, 120, 0);
      return;
    case Mode::BOUNCE:
      fillMatrix(0, 130, 170);
      return;
    case Mode::IDLE:
    default:
      break;
  }

  // IDLE breathing effect: sinusoidal pulse on white
  // Use full 0-255 range to preserve smoothness at low brightness settings
  float breathePhase = static_cast<float>(now % IDLE_BREATHING_CYCLE_MS) / static_cast<float>(IDLE_BREATHING_CYCLE_MS);
  float breatheWave = (sinf(breathePhase * 2.0f * PI) + 1.0f) * 0.5f;
  uint8_t breatheLevel = static_cast<uint8_t>(100.0f + breatheWave * 155.0f);  // 100-255 range
  uint8_t idleBlue = static_cast<uint8_t>((static_cast<uint16_t>(breatheLevel) * pulseProfile.blueScale) / 255U);
  fillMatrix(breatheLevel, breatheLevel, idleBlue);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  prefs.begin(PREF_NS, false);
  uint8_t savedBrightness = prefs.getUChar(PREF_KEY_BRIGHT, systemState.brightness);
  if (savedBrightness <= 100U) {
    systemState.brightness = savedBrightness;
  }

  randomSeed(static_cast<uint32_t>(micros() ^ millis()));

  matrixStrip.begin();
  matrixStrip.setBrightness(1);
  matrixStrip.clear();
  matrixStrip.show();

  systemState.lastStatusRxMs = millis();
  resetRandomTwinkleSchedule(systemState.lastStatusRxMs, 70, 170);

  Serial.println("ESP32-S3 Matrix status listener started (state-driven)");
  Serial.print("UART1 RX=");
  Serial.print(UART_RX_PIN);
  Serial.print(" TX=");
  Serial.println(UART_TX_PIN);
}

void loop() {
  processUartInput();
  const unsigned long now = millis();
  updateState(systemState, now);
  renderCurrentState(systemState);
}

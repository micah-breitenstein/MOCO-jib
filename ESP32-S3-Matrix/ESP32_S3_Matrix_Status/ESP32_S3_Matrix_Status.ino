#include <Adafruit_NeoPixel.h>

constexpr uint8_t UART_RX_PIN = 44;
constexpr uint8_t UART_TX_PIN = 43;
constexpr uint32_t UART_BAUD = 9600;

// Set this to the actual onboard RGB matrix data pin for your Waveshare board.
constexpr uint8_t MATRIX_DATA_PIN = 14;
constexpr uint16_t MATRIX_LED_COUNT = 64;

Adafruit_NeoPixel matrixStrip(MATRIX_LED_COUNT, MATRIX_DATA_PIN, NEO_RGB + NEO_KHZ800);

String lineBuffer;
bool errorActive = false;
unsigned long lastTwinkleMs = 0;
unsigned long lastStatusRxMs = 0;
unsigned long lastOkPulseMs = 0;
uint8_t lastOkPulseLevel = 0;

constexpr unsigned long TWINKLE_FAST_PHASE_MS = 3200;
constexpr unsigned long TWINKLE_SLOW_PHASE_MS = 1200;
constexpr unsigned long STATUS_SIGNAL_TIMEOUT_MS = 3500;
constexpr unsigned long TWINKLE_UPDATE_FAST_MS = 14;
constexpr unsigned long TWINKLE_UPDATE_SLOW_MS = 28;
constexpr unsigned long TWINKLE_CYCLE_FAST_MS = 360;
constexpr unsigned long TWINKLE_CYCLE_SLOW_MS = 620;
constexpr unsigned long OK_PULSE_CYCLE_MS = 1800;
constexpr uint8_t OK_PULSE_MIN_LEVEL = 50;
constexpr uint8_t OK_PULSE_MAX_LEVEL = 64;
constexpr uint8_t BASE_ERROR_RED = 200;
constexpr uint8_t TWINKLE_WHITE_PEAK = 200;
constexpr uint8_t RANDOM_TWINKLE_CHANCE_PERCENT = 22;
constexpr unsigned long RANDOM_TWINKLE_MIN_MS = 90;
constexpr unsigned long RANDOM_TWINKLE_MAX_MS = 260;
constexpr unsigned long RANDOM_TWINKLE_GAP_MIN_MS = 35;
constexpr unsigned long RANDOM_TWINKLE_GAP_MAX_MS = 420;
constexpr uint8_t RANDOM_TWINKLE_PEAK_MIN = 70;
constexpr uint8_t RANDOM_TWINKLE_PEAK_MAX = 170;

unsigned long randomTwinkleStartMs[MATRIX_LED_COUNT] = {0};
unsigned long randomTwinkleEndMs[MATRIX_LED_COUNT] = {0};
unsigned long randomTwinkleNextMs[MATRIX_LED_COUNT] = {0};
uint8_t randomTwinklePeak[MATRIX_LED_COUNT] = {0};

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
    if (i % 3 == 0) {
      continue;
    }

    if (nowMs < randomTwinkleNextMs[i]) {
      continue;
    }

    if (random(100) < RANDOM_TWINKLE_CHANCE_PERCENT) {
      randomTwinkleStartMs[i] = nowMs;
      randomTwinkleEndMs[i] = nowMs + (unsigned long)random(RANDOM_TWINKLE_MIN_MS, RANDOM_TWINKLE_MAX_MS + 1);
      randomTwinklePeak[i] = (uint8_t)random(RANDOM_TWINKLE_PEAK_MIN, RANDOM_TWINKLE_PEAK_MAX + 1);
    }

    randomTwinkleNextMs[i] = nowMs + (unsigned long)random(RANDOM_TWINKLE_GAP_MIN_MS, RANDOM_TWINKLE_GAP_MAX_MS + 1);
  }
}

void renderErrorTwinkleFrame(unsigned long nowMs, unsigned long twinkleCycleMs) {
  const unsigned long halfCycle = twinkleCycleMs / 2;

  for (uint16_t i = 0; i < MATRIX_LED_COUNT; ++i) {
    uint8_t r = BASE_ERROR_RED;
    uint8_t g = 0;
    uint8_t b = 0;

    if (i % 3 == 0) {
      unsigned long phase = (nowMs + (i * 83UL)) % twinkleCycleMs;
      unsigned long ramp = (phase <= halfCycle) ? phase : (twinkleCycleMs - phase);
      uint8_t white = (uint8_t)((ramp * TWINKLE_WHITE_PEAK) / halfCycle);

      uint16_t redMixed = (uint16_t)BASE_ERROR_RED + white;
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

      uint16_t redMixed = (uint16_t)BASE_ERROR_RED + white;
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

void showError() {
  errorActive = true;
  lastStatusRxMs = millis();
  resetRandomTwinkleSchedule(lastStatusRxMs);
  renderErrorTwinkleFrame(millis(), TWINKLE_CYCLE_FAST_MS);
}

void showOk() {
  errorActive = false;
  lastStatusRxMs = millis();
  lastOkPulseMs = lastStatusRxMs;
  lastOkPulseLevel = 0;
  fillMatrix(OK_PULSE_MIN_LEVEL, OK_PULSE_MIN_LEVEL, OK_PULSE_MIN_LEVEL);
}

void handleStatusLine(const String& line) {
  if (line.startsWith("CONTROLLER_ERROR:")) {
    showError();
    Serial.println("Matrix status: ERROR -> RED");
    return;
  }

  if (line.startsWith("CONTROLLER_OK:")) {
    showOk();
    Serial.println("Matrix status: OK -> OFF");
    return;
  }

  if (line.startsWith("MODE:")) {
    Serial.print("Matrix received mode update: ");
    Serial.println(line);
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
  matrixStrip.setBrightness(64);
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

  if (errorActive && (now - lastStatusRxMs > STATUS_SIGNAL_TIMEOUT_MS)) {
    showOk();
    Serial.println("Matrix status timeout -> OK (dim green)");
  }

  if (errorActive) {
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
  } else {
    unsigned long phase = now % OK_PULSE_CYCLE_MS;
    unsigned long halfCycle = OK_PULSE_CYCLE_MS / 2;
    unsigned long ramp = (phase <= halfCycle) ? phase : (OK_PULSE_CYCLE_MS - phase);

    if (halfCycle == 0) {
      halfCycle = 1;
    }

    uint8_t pulseLevel = OK_PULSE_MIN_LEVEL +
                         (uint8_t)((ramp * (OK_PULSE_MAX_LEVEL - OK_PULSE_MIN_LEVEL)) / halfCycle);

    if (now - lastOkPulseMs >= 20) {
      lastOkPulseMs = now;
      if (pulseLevel != lastOkPulseLevel) {
        lastOkPulseLevel = pulseLevel;
        fillMatrix(pulseLevel, pulseLevel, pulseLevel);
      }
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

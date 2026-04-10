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

constexpr unsigned long TWINKLE_FAST_PHASE_MS = 3200;
constexpr unsigned long TWINKLE_SLOW_PHASE_MS = 1200;
constexpr unsigned long STATUS_SIGNAL_TIMEOUT_MS = 3500;
constexpr unsigned long TWINKLE_UPDATE_FAST_MS = 14;
constexpr unsigned long TWINKLE_UPDATE_SLOW_MS = 28;
constexpr unsigned long TWINKLE_CYCLE_FAST_MS = 360;
constexpr unsigned long TWINKLE_CYCLE_SLOW_MS = 620;
constexpr uint8_t BASE_ERROR_RED = 200;
constexpr uint8_t TWINKLE_WHITE_PEAK = 200;

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
  renderErrorTwinkleFrame(millis(), TWINKLE_CYCLE_FAST_MS);
}

void showOk() {
  errorActive = false;
  lastStatusRxMs = millis();
  fillMatrix(14, 14, 14);
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

  matrixStrip.begin();
  matrixStrip.setBrightness(32);
  fillMatrix(0, 64, 0);
  delay(300);
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
    const unsigned long phaseSpan = TWINKLE_FAST_PHASE_MS + TWINKLE_SLOW_PHASE_MS;
    const unsigned long phasePos = now % phaseSpan;
    bool fastPhase = phasePos < TWINKLE_FAST_PHASE_MS;
    unsigned long updateInterval = fastPhase ? TWINKLE_UPDATE_FAST_MS : TWINKLE_UPDATE_SLOW_MS;
    unsigned long twinkleCycle = fastPhase ? TWINKLE_CYCLE_FAST_MS : TWINKLE_CYCLE_SLOW_MS;

    if (now - lastTwinkleMs >= updateInterval) {
      lastTwinkleMs = now;
      renderErrorTwinkleFrame(now, twinkleCycle);
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

#include <PS2X_lib.h>  //for v1.6

constexpr uint8_t PS2_DAT = 10;
constexpr uint8_t PS2_CMD = 9;
constexpr uint8_t PS2_SEL = 8;
constexpr uint8_t PS2_CLK = 11;

constexpr bool PRESSURES = false;
constexpr bool RUMBLE = true;
constexpr bool DEBUG_EDGE_EVENTS = true;

PS2X ps2x;
int error = 0;
byte controllerType = 0;
byte vibrate = 0;

// Output pins and control state
const uint8_t swingLeft = 24;
const uint8_t swingRight = 25;
const uint8_t swingSpeedUp = 26;
const uint8_t swingSpeedDown = 27;
int swingSoloMode = 0;

const uint8_t panLeft = 46;
const uint8_t panRight = 48;
const uint8_t panSpeedUp = 50;
const uint8_t panSpeedDown = 52;
int panStop = 0;

const uint8_t panSpeedUpOnly = 29; //lower
const uint8_t panSpeedDownOnly = 45; //higher

const uint8_t liftDown = 30;
const uint8_t liftUp = 31;
const uint8_t liftSpeedUp = 32;
const uint8_t liftSpeedDown = 33;
int liftSoloMode = 0;

const uint8_t tiltSpeedUpOnly = 42; //lower val
const uint8_t tiltSpeedDownOnly = 44; //higher val

const uint8_t tiltDown = 34;
const uint8_t tiltUp = 36;
const uint8_t tiltSpeedUp = 38;
const uint8_t tiltSpeedDown = 40;
int tiltStop = 0;

const uint8_t focusLeft = 47;
const uint8_t focusRight = 49;
const uint8_t focusSpeedUp = 51;
const uint8_t focusSpeedDown = 53;

bool swingInMotion = false;
bool liftInMotion = false;

int leftStickXvalue;
int leftStickYvalue;
int rightStickXvalue;
int rightStickYvalue;

// Timelapse Variables
int timelapseMode = 0;
int timelapseIntervalSeconds = 15;
unsigned long timelapseIntervalMs;
int stepDist = 100;
const uint8_t trigger = 28;

enum TimelapsePhase {
  TIMELAPSE_PHASE_IDLE,
  TIMELAPSE_PHASE_TRIGGER_LOW,
  TIMELAPSE_PHASE_TRIGGER_HIGH,
  TIMELAPSE_PHASE_MOVE_ACTIVE
};

TimelapsePhase timelapsePhase = TIMELAPSE_PHASE_IDLE;
unsigned long timelapsePhaseStartMs = 0;

enum IntervalRumblePhase {
  INTERVAL_RUMBLE_IDLE,
  INTERVAL_RUMBLE_LONG_ACTIVE,
  INTERVAL_RUMBLE_SHORT_ACTIVE,
  INTERVAL_RUMBLE_SHORT_PAUSE
};

enum StepDistRumblePhase {
  STEP_DIST_RUMBLE_IDLE,
  STEP_DIST_RUMBLE_LONG_ACTIVE,
  STEP_DIST_RUMBLE_LONG_PAUSE
};

enum PendingRumblePattern {
  PENDING_RUMBLE_NONE,
  PENDING_RUMBLE_INTERVAL,
  PENDING_RUMBLE_STEP_DIST
};

enum FeedbackRumblePhase {
  FEEDBACK_RUMBLE_IDLE,
  FEEDBACK_RUMBLE_ON,
  FEEDBACK_RUMBLE_PAUSE
};

IntervalRumblePhase intervalRumblePhase = INTERVAL_RUMBLE_IDLE;
unsigned long intervalRumblePhaseStartMs = 0;
uint8_t intervalRumbleLongsRemaining = 0;
uint8_t intervalRumbleShortsRemaining = 0;
StepDistRumblePhase stepDistRumblePhase = STEP_DIST_RUMBLE_IDLE;
unsigned long stepDistRumblePhaseStartMs = 0;
uint8_t stepDistRumbleLongsRemaining = 0;
PendingRumblePattern pendingRumblePattern = PENDING_RUMBLE_NONE;
bool rumbleSeparatorActive = false;
unsigned long rumbleSeparatorStartMs = 0;
FeedbackRumblePhase feedbackRumblePhase = FEEDBACK_RUMBLE_IDLE;
unsigned long feedbackRumblePhaseStartMs = 0;
uint8_t feedbackRumblePulsesRemaining = 0;

// Motion Control Variables
int bounce = 0;
int stage = 0;
unsigned long bouncePhaseStartMs = 0;
unsigned long bounceMoveDurationMs = 0;

constexpr uint8_t DIP_SWITCH_1 = 35;
constexpr uint8_t DIP_SWITCH_2 = 43;
constexpr uint8_t DIP_SWITCH_3 = 37;
constexpr uint8_t DIP_SWITCH_4 = 39;
constexpr uint8_t DIP_SWITCH_5 = 41;
constexpr unsigned long CONTROLLER_STARTUP_DELAY_MS = 300;
constexpr int STICK_MIN = 0;
constexpr int STICK_CENTER = 128;
constexpr int STICK_MAX = 255;
constexpr int STICK_MODE_LOW_THRESHOLD = 123;
constexpr int STICK_MODE_HIGH_THRESHOLD = 133;
constexpr int PAN_STOP_NONE = 0;
constexpr int PAN_STOP_ACTIVE = 1;
constexpr int PAN_STOP_TRIM = 2;
constexpr int TILT_STOP_NONE = 0;
constexpr int TILT_STOP_ACTIVE = 1;
constexpr int TILT_STOP_TRIM = 2;
constexpr int TIMELAPSE_INTERVAL_MIN_SECONDS = 1;
constexpr int TIMELAPSE_INTERVAL_MAX_SECONDS = 99;
constexpr int TIMELAPSE_STEP_DIST_MIN_MS = 20;
constexpr int TIMELAPSE_STEP_DIST_MAX_MS = 150;
constexpr int TIMELAPSE_STEP_DIST_ADJUST_INCREMENT_MS = 10;
constexpr uint8_t RUMBLE_ACTIVE_INTENSITY = 255;
constexpr unsigned long INTERVAL_RUMBLE_LONG_MS = 10000;
constexpr unsigned long INTERVAL_RUMBLE_SHORT_ON_MS = 200;
constexpr unsigned long INTERVAL_RUMBLE_SHORT_TOTAL_MS = 1000;
constexpr unsigned long STEP_DIST_RUMBLE_LONG_ON_MS = 600;
constexpr unsigned long STEP_DIST_RUMBLE_LONG_TOTAL_MS = 1000;
constexpr unsigned long RUMBLE_PATTERN_SEPARATOR_MS = 300;
constexpr unsigned long FEEDBACK_RUMBLE_ON_MS = 120;
constexpr unsigned long FEEDBACK_RUMBLE_TOTAL_MS = 240;
constexpr uint8_t LIMIT_REACHED_RUMBLE_PULSES = 2;

bool isSwingReversed = false;
bool isPanReversed = false;
bool isLiftReversed = false;
bool isTiltReversed = false;
bool isFocusReversed = false;
bool lastIntervalAdjustUpComboActive = false;
bool lastIntervalAdjustDownComboActive = false;
bool lastStepDistAdjustUpComboActive = false;
bool lastStepDistAdjustDownComboActive = false;
bool lastEmergencyStopComboActive = false;

void configureController() {

  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, PRESSURES, RUMBLE);

  if (error == 0) {
    Serial.print("Found Controller, configured successful ");
    Serial.print("pressures = ");
    if (PRESSURES)
      Serial.println("true ");
    else
      Serial.println("false");
    Serial.print("rumble = ");
    if (RUMBLE)
      Serial.println("true)");
    else
      Serial.println("false");
    Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
    Serial.println("holding L1 or R1 will print out the analog stick values.");
    Serial.println("Note: Go to www.billporter.info for updates and to report bugs.");
  }
  else if (error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");

  else if (error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

  else if (error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");

  //  Serial.print(ps2x.Analog(1), HEX);
}

void detectControllerType() {

  controllerType = ps2x.readType();
  switch (controllerType) {
    case 0:
      Serial.print("Unknown Controller type found ");
      break;
    case 1:
      Serial.print("DualShock Controller found ");
      break;
    case 2:
      Serial.print("GuitarHero Controller found ");
      break;
    case 3:
      Serial.print("Wireless Sony DualShock Controller found ");
      break;
  }
}

void handleAxisSpeedControl(uint8_t buttonCode, uint8_t axis1Pin, uint8_t axis2Pin) {
  if (ps2x.Button(buttonCode)) {
    digitalWrite(axis1Pin, HIGH);
    digitalWrite(axis2Pin, HIGH);
  }
  if (ps2x.ButtonReleased(buttonCode)) {
    digitalWrite(axis1Pin, LOW);
    digitalWrite(axis2Pin, LOW);
  }
}

void setDirectionalOutput(bool isReversed, uint8_t normalPin, uint8_t reversedPin, uint8_t state) {
  if (!isReversed) {
    digitalWrite(normalPin, state);
  } else {
    digitalWrite(reversedPin, state);
  }
}

void handleSoloDirectionalMode(uint8_t buttonCode, bool isReversed, uint8_t normalPin, uint8_t reversedPin, int& modeState) {
  if (ps2x.Button(PSB_SELECT) && ps2x.Button(buttonCode)) {
    if (DEBUG_EDGE_EVENTS && (buttonCode == PSB_PAD_LEFT || buttonCode == PSB_PAD_RIGHT)) {
      Serial.print("SOLO press: ");
      Serial.println(buttonCode == PSB_PAD_LEFT ? "PAD_LEFT" : "PAD_RIGHT");
    }
    setDirectionalOutput(isReversed, normalPin, reversedPin, HIGH);
    modeState = 1;
  }
  if (modeState == 1 && ps2x.ButtonReleased(buttonCode)) {
    if (DEBUG_EDGE_EVENTS && (buttonCode == PSB_PAD_LEFT || buttonCode == PSB_PAD_RIGHT)) {
      Serial.print("SOLO release: ");
      Serial.println(buttonCode == PSB_PAD_LEFT ? "PAD_LEFT" : "PAD_RIGHT");
    }
    digitalWrite(normalPin, LOW);
    digitalWrite(reversedPin, LOW);
    modeState = 0;
  }
}

void handleCombinedDirectionalMode(uint8_t buttonCode, bool axis1Reversed, uint8_t axis1Normal, uint8_t axis1Rev,
                                    bool axis2Reversed, uint8_t axis2Normal, uint8_t axis2Rev,
                                    int& soloState, bool& motionState) {
  if (soloState == 0 && ps2x.Button(buttonCode)) {
    if (DEBUG_EDGE_EVENTS && (buttonCode == PSB_PAD_LEFT || buttonCode == PSB_PAD_RIGHT)) {
      Serial.print("COMBINED press: ");
      Serial.println(buttonCode == PSB_PAD_LEFT ? "PAD_LEFT" : "PAD_RIGHT");
    }
    motionState = true;
    setDirectionalOutput(axis1Reversed, axis1Normal, axis1Rev, HIGH);
    setDirectionalOutput(axis2Reversed, axis2Normal, axis2Rev, HIGH);
  }
  if (soloState == 0 && ps2x.ButtonReleased(buttonCode)) {
    if (DEBUG_EDGE_EVENTS && (buttonCode == PSB_PAD_LEFT || buttonCode == PSB_PAD_RIGHT)) {
      Serial.print("COMBINED release: ");
      Serial.println(buttonCode == PSB_PAD_LEFT ? "PAD_LEFT" : "PAD_RIGHT");
    }
    digitalWrite(axis1Normal, LOW);
    digitalWrite(axis1Rev, LOW);
    digitalWrite(axis2Normal, LOW);
    digitalWrite(axis2Rev, LOW);
    motionState = false;
  }
}

void handleSwingOnly(uint8_t buttonCode, uint8_t swingNormalPin, uint8_t swingReversedPin) {
  handleSoloDirectionalMode(buttonCode, isSwingReversed, swingNormalPin, swingReversedPin, swingSoloMode);
}

void handleSwingAndPan(uint8_t buttonCode, uint8_t swingNormalPin, uint8_t swingReversedPin,
                        uint8_t panNormalPin, uint8_t panReversedPin) {
  handleCombinedDirectionalMode(buttonCode, isSwingReversed, swingNormalPin, swingReversedPin,
                                 isPanReversed, panNormalPin, panReversedPin, swingSoloMode, swingInMotion);
}

void activatePanTrim(uint8_t pin, const char* label) {
  digitalWrite(pin, HIGH);
  Serial.println(label);
  panStop = PAN_STOP_TRIM;
}

void applyPanTrimDuringSwing() {
  if (swingInMotion) {
    if (rightStickXvalue == STICK_MIN) {
      activatePanTrim(panSpeedDownOnly, "panSpeedDownOnly");
    } else if (rightStickXvalue == STICK_MAX) {
      activatePanTrim(panSpeedUpOnly, "panSpeedUpOnly");
    }
  }
}

void resetPanTrimAtCenter() {
  if (panStop == PAN_STOP_TRIM && rightStickXvalue == STICK_CENTER) {
    digitalWrite(panSpeedUpOnly, LOW);
    digitalWrite(panSpeedDownOnly, LOW);
    panStop = PAN_STOP_NONE;
  }
}

void handlePanTrimAxis() {
  applyPanTrimDuringSwing();
  resetPanTrimAtCenter();
}

void activatePanOnlyMotion(uint8_t normalPin, uint8_t reversedPin, const char* label) {
  Serial.println(label);
  digitalWrite(panSpeedUpOnly, HIGH); //signal to nano to use top speed
  digitalWrite(panSpeedDownOnly, HIGH); //signal to nano to use top speed
  setDirectionalOutput(isPanReversed, normalPin, reversedPin, HIGH);
  panStop = PAN_STOP_ACTIVE;
}

void stopPanOnlyMotionAtCenter() {
  if (!swingInMotion && panStop == PAN_STOP_ACTIVE && rightStickXvalue == STICK_CENTER) {
    digitalWrite(panLeft, LOW);
    digitalWrite(panRight, LOW);
    digitalWrite(panSpeedUpOnly, LOW);
    digitalWrite(panSpeedDownOnly, LOW);
    panStop = PAN_STOP_NONE;
  }
}

void handlePanAxis() {
  if (!swingInMotion) {
    if (rightStickXvalue == STICK_MIN) {
      activatePanOnlyMotion(panLeft, panRight, "panleftnonly with top speed");
    } else if (rightStickXvalue == STICK_MAX) {
      activatePanOnlyMotion(panRight, panLeft, "panrightonly with top speed");
    }
  }

  stopPanOnlyMotionAtCenter();
}

void handleLiftOnly(uint8_t buttonCode, uint8_t liftNormalPin, uint8_t liftReversedPin) {
  handleSoloDirectionalMode(buttonCode, isLiftReversed, liftNormalPin, liftReversedPin, liftSoloMode);
}

void handleTiltTrimAxis() {
  if (liftInMotion && rightStickYvalue == STICK_MAX) {
    Serial.println("tiltSpeedDownOnly");
    digitalWrite(tiltSpeedDownOnly, HIGH);
    tiltStop = TILT_STOP_TRIM;
  }
  if (liftInMotion && rightStickYvalue == STICK_MIN) {
    Serial.println("tiltSpeedUpOnly");
    digitalWrite(tiltSpeedUpOnly, HIGH);
    tiltStop = TILT_STOP_TRIM;
  }
  if (tiltStop == TILT_STOP_TRIM && rightStickYvalue == STICK_CENTER) {
    digitalWrite(tiltSpeedUpOnly, LOW);
    digitalWrite(tiltSpeedDownOnly, LOW);
    tiltStop = TILT_STOP_NONE;
  }
}

void handleLiftAndTilt(uint8_t buttonCode, uint8_t liftNormalPin, uint8_t liftReversedPin,
                       uint8_t tiltNormalPin, uint8_t tiltReversedPin) {
  handleCombinedDirectionalMode(buttonCode, isLiftReversed, liftNormalPin, liftReversedPin,
                                isTiltReversed, tiltNormalPin, tiltReversedPin, liftSoloMode, liftInMotion);
}

void activateTiltOnlyMotion(uint8_t normalPin, uint8_t reversedPin, const char* label) {
  Serial.println(label);
  digitalWrite(tiltSpeedUpOnly, HIGH);
  digitalWrite(tiltSpeedDownOnly, HIGH);
  setDirectionalOutput(isTiltReversed, normalPin, reversedPin, HIGH);
  tiltStop = TILT_STOP_ACTIVE;
}

void stopTiltOnlyMotionAtCenter() {
  if (!liftInMotion && tiltStop == TILT_STOP_ACTIVE && rightStickYvalue == STICK_CENTER) {
    digitalWrite(tiltUp, LOW);
    digitalWrite(tiltDown, LOW);
    digitalWrite(tiltSpeedUpOnly, LOW);
    digitalWrite(tiltSpeedDownOnly, LOW);
    tiltStop = TILT_STOP_NONE;
  }
}

void handleTiltAxis() {
  if (!liftInMotion) {
    if (rightStickYvalue == STICK_MAX) {
      activateTiltOnlyMotion(tiltDown, tiltUp, "tiltDownOnly");
    } else if (rightStickYvalue == STICK_MIN) {
      activateTiltOnlyMotion(tiltUp, tiltDown, "tiltUpOnly");
    }
  }

  stopTiltOnlyMotionAtCenter();
}

void handleButtonDirection(uint8_t buttonCode, bool isReversed,
                           uint8_t normalPin, uint8_t reversedPin) {
  if (ps2x.Button(buttonCode)) {
    setDirectionalOutput(isReversed, normalPin, reversedPin, HIGH);
  }
  if (ps2x.ButtonReleased(buttonCode)) {
    digitalWrite(normalPin, LOW);
    digitalWrite(reversedPin, LOW);
  }
}

void handleFocusAxis() {
  handleButtonDirection(PSB_TRIANGLE, isFocusReversed, focusLeft, focusRight);
  handleButtonDirection(PSB_CROSS, isFocusReversed, focusRight, focusLeft);
  handleAxisSpeedControl(PSB_SQUARE, focusSpeedDown, focusSpeedDown);
  handleAxisSpeedControl(PSB_CIRCLE, focusSpeedUp, focusSpeedUp);
}

void stopAllMotors() {
  digitalWrite(swingLeft, LOW);
  digitalWrite(swingRight, LOW);
  digitalWrite(panLeft, LOW);
  digitalWrite(panRight, LOW);
  digitalWrite(panSpeedUpOnly, LOW);
  digitalWrite(panSpeedDownOnly, LOW);
  digitalWrite(liftUp, LOW);
  digitalWrite(liftDown, LOW);
  digitalWrite(tiltUp, LOW);
  digitalWrite(tiltDown, LOW);
  digitalWrite(tiltSpeedUpOnly, LOW);
  digitalWrite(tiltSpeedDownOnly, LOW);
}

void updateIntervalMs() {
  timelapseIntervalMs = static_cast<unsigned long>(timelapseIntervalSeconds) * 1000UL;
}

void stopIntervalRumbleFeedback() {
  intervalRumblePhase = INTERVAL_RUMBLE_IDLE;
  intervalRumblePhaseStartMs = 0;
  intervalRumbleLongsRemaining = 0;
  intervalRumbleShortsRemaining = 0;
  stepDistRumblePhase = STEP_DIST_RUMBLE_IDLE;
  stepDistRumblePhaseStartMs = 0;
  stepDistRumbleLongsRemaining = 0;
  pendingRumblePattern = PENDING_RUMBLE_NONE;
  rumbleSeparatorActive = false;
  rumbleSeparatorStartMs = 0;
  feedbackRumblePhase = FEEDBACK_RUMBLE_IDLE;
  feedbackRumblePhaseStartMs = 0;
  feedbackRumblePulsesRemaining = 0;
  vibrate = 0;
}

void startLimitReachedRumbleFeedback() {
  stopIntervalRumbleFeedback();
  feedbackRumblePhase = FEEDBACK_RUMBLE_ON;
  feedbackRumblePhaseStartMs = millis();
  feedbackRumblePulsesRemaining = LIMIT_REACHED_RUMBLE_PULSES;
  vibrate = RUMBLE_ACTIVE_INTENSITY;
}

void startIntervalRumbleFeedbackNow() {
  stepDistRumblePhase = STEP_DIST_RUMBLE_IDLE;
  stepDistRumblePhaseStartMs = 0;
  stepDistRumbleLongsRemaining = 0;

  intervalRumbleLongsRemaining = timelapseIntervalSeconds / 10;
  intervalRumbleShortsRemaining = timelapseIntervalSeconds % 10;

  if (intervalRumbleLongsRemaining > 0) {
    intervalRumblePhase = INTERVAL_RUMBLE_LONG_ACTIVE;
  } else if (intervalRumbleShortsRemaining > 0) {
    intervalRumblePhase = INTERVAL_RUMBLE_SHORT_ACTIVE;
  } else {
    stopIntervalRumbleFeedback();
    return;
  }

  intervalRumblePhaseStartMs = millis();
  vibrate = RUMBLE_ACTIVE_INTENSITY;
}

void startStepDistRumbleFeedbackNow() {
  intervalRumblePhase = INTERVAL_RUMBLE_IDLE;
  intervalRumblePhaseStartMs = 0;
  intervalRumbleLongsRemaining = 0;
  intervalRumbleShortsRemaining = 0;

  stepDistRumbleLongsRemaining = stepDist / TIMELAPSE_STEP_DIST_ADJUST_INCREMENT_MS;
  if (stepDistRumbleLongsRemaining == 0) {
    stopIntervalRumbleFeedback();
    return;
  }

  stepDistRumblePhase = STEP_DIST_RUMBLE_LONG_ACTIVE;
  stepDistRumblePhaseStartMs = millis();
  vibrate = RUMBLE_ACTIVE_INTENSITY;
}

void startRumbleSeparator(PendingRumblePattern nextPattern) {
  intervalRumblePhase = INTERVAL_RUMBLE_IDLE;
  intervalRumblePhaseStartMs = 0;
  intervalRumbleLongsRemaining = 0;
  intervalRumbleShortsRemaining = 0;
  stepDistRumblePhase = STEP_DIST_RUMBLE_IDLE;
  stepDistRumblePhaseStartMs = 0;
  stepDistRumbleLongsRemaining = 0;
  pendingRumblePattern = nextPattern;
  rumbleSeparatorActive = true;
  rumbleSeparatorStartMs = millis();
  vibrate = 0;
}

void startIntervalRumbleFeedback() {
  if (stepDistRumblePhase != STEP_DIST_RUMBLE_IDLE) {
    startRumbleSeparator(PENDING_RUMBLE_INTERVAL);
    return;
  }

  pendingRumblePattern = PENDING_RUMBLE_NONE;
  rumbleSeparatorActive = false;
  rumbleSeparatorStartMs = 0;
  startIntervalRumbleFeedbackNow();
}

void startStepDistRumbleFeedback() {
  if (intervalRumblePhase != INTERVAL_RUMBLE_IDLE) {
    startRumbleSeparator(PENDING_RUMBLE_STEP_DIST);
    return;
  }

  pendingRumblePattern = PENDING_RUMBLE_NONE;
  rumbleSeparatorActive = false;
  rumbleSeparatorStartMs = 0;
  startStepDistRumbleFeedbackNow();
}

void handleIntervalRumbleFeedback(unsigned long now) {
  if (feedbackRumblePhase != FEEDBACK_RUMBLE_IDLE) {
    switch (feedbackRumblePhase) {
      case FEEDBACK_RUMBLE_ON:
        vibrate = RUMBLE_ACTIVE_INTENSITY;
        if (now - feedbackRumblePhaseStartMs >= FEEDBACK_RUMBLE_ON_MS) {
          feedbackRumblePhase = FEEDBACK_RUMBLE_PAUSE;
          feedbackRumblePhaseStartMs = now;
          vibrate = 0;
        }
        break;
      case FEEDBACK_RUMBLE_PAUSE:
        vibrate = 0;
        if (now - feedbackRumblePhaseStartMs >= (FEEDBACK_RUMBLE_TOTAL_MS - FEEDBACK_RUMBLE_ON_MS)) {
          if (feedbackRumblePulsesRemaining > 0) {
            feedbackRumblePulsesRemaining--;
          }
          if (feedbackRumblePulsesRemaining > 0) {
            feedbackRumblePhase = FEEDBACK_RUMBLE_ON;
            feedbackRumblePhaseStartMs = now;
          } else {
            stopIntervalRumbleFeedback();
          }
        }
        break;
      case FEEDBACK_RUMBLE_IDLE:
        break;
    }
    return;
  }

  if (rumbleSeparatorActive) {
    vibrate = 0;
    if (now - rumbleSeparatorStartMs >= RUMBLE_PATTERN_SEPARATOR_MS) {
      rumbleSeparatorActive = false;
      rumbleSeparatorStartMs = 0;
      PendingRumblePattern patternToStart = pendingRumblePattern;
      pendingRumblePattern = PENDING_RUMBLE_NONE;

      if (patternToStart == PENDING_RUMBLE_INTERVAL) {
        startIntervalRumbleFeedbackNow();
      } else if (patternToStart == PENDING_RUMBLE_STEP_DIST) {
        startStepDistRumbleFeedbackNow();
      }
    }
    return;
  }

  if (stepDistRumblePhase != STEP_DIST_RUMBLE_IDLE) {
    switch (stepDistRumblePhase) {
      case STEP_DIST_RUMBLE_LONG_ACTIVE:
        vibrate = RUMBLE_ACTIVE_INTENSITY;
        if (now - stepDistRumblePhaseStartMs >= STEP_DIST_RUMBLE_LONG_ON_MS) {
          stepDistRumblePhase = STEP_DIST_RUMBLE_LONG_PAUSE;
          stepDistRumblePhaseStartMs = now;
          vibrate = 0;
        }
        break;
      case STEP_DIST_RUMBLE_LONG_PAUSE:
        vibrate = 0;
        if (now - stepDistRumblePhaseStartMs >= (STEP_DIST_RUMBLE_LONG_TOTAL_MS - STEP_DIST_RUMBLE_LONG_ON_MS)) {
          if (stepDistRumbleLongsRemaining > 0) {
            stepDistRumbleLongsRemaining--;
          }
          if (stepDistRumbleLongsRemaining > 0) {
            stepDistRumblePhase = STEP_DIST_RUMBLE_LONG_ACTIVE;
            stepDistRumblePhaseStartMs = now;
          } else {
            stopIntervalRumbleFeedback();
          }
        }
        break;
      case STEP_DIST_RUMBLE_IDLE:
        break;
    }
    return;
  }

  switch (intervalRumblePhase) {
    case INTERVAL_RUMBLE_IDLE:
      vibrate = 0;
      break;
    case INTERVAL_RUMBLE_LONG_ACTIVE:
      vibrate = RUMBLE_ACTIVE_INTENSITY;
      if (now - intervalRumblePhaseStartMs >= INTERVAL_RUMBLE_LONG_MS) {
        intervalRumbleLongsRemaining--;
        if (intervalRumbleLongsRemaining > 0) {
          intervalRumblePhaseStartMs = now;
        } else if (intervalRumbleShortsRemaining > 0) {
          intervalRumblePhase = INTERVAL_RUMBLE_SHORT_ACTIVE;
          intervalRumblePhaseStartMs = now;
        } else {
          stopIntervalRumbleFeedback();
        }
      }
      break;
    case INTERVAL_RUMBLE_SHORT_ACTIVE:
      vibrate = RUMBLE_ACTIVE_INTENSITY;
      if (now - intervalRumblePhaseStartMs >= INTERVAL_RUMBLE_SHORT_ON_MS) {
        intervalRumblePhase = INTERVAL_RUMBLE_SHORT_PAUSE;
        intervalRumblePhaseStartMs = now;
        vibrate = 0;
      }
      break;
    case INTERVAL_RUMBLE_SHORT_PAUSE:
      vibrate = 0;
      if (now - intervalRumblePhaseStartMs >= (INTERVAL_RUMBLE_SHORT_TOTAL_MS - INTERVAL_RUMBLE_SHORT_ON_MS)) {
        intervalRumbleShortsRemaining--;
        if (intervalRumbleShortsRemaining > 0) {
          intervalRumblePhase = INTERVAL_RUMBLE_SHORT_ACTIVE;
          intervalRumblePhaseStartMs = now;
        } else {
          stopIntervalRumbleFeedback();
        }
      }
      break;
  }
}

void adjustIntervalSeconds(int delta) {
  int newIntervalSeconds = timelapseIntervalSeconds + delta;
  if (newIntervalSeconds < TIMELAPSE_INTERVAL_MIN_SECONDS) {
    newIntervalSeconds = TIMELAPSE_INTERVAL_MIN_SECONDS;
  }
  if (newIntervalSeconds > TIMELAPSE_INTERVAL_MAX_SECONDS) {
    newIntervalSeconds = TIMELAPSE_INTERVAL_MAX_SECONDS;
  }
  if (newIntervalSeconds == timelapseIntervalSeconds) {
    startLimitReachedRumbleFeedback();
    Serial.println("Timelapse interval limit reached.");
    return;
  }

  timelapseIntervalSeconds = newIntervalSeconds;
  updateIntervalMs();
  Serial.print("Timelapse interval (seconds) = ");
  Serial.println(timelapseIntervalSeconds);
  startIntervalRumbleFeedback();
}

void adjustStepDist(int delta) {
  int newStepDist = stepDist + delta;
  if (newStepDist < TIMELAPSE_STEP_DIST_MIN_MS) {
    newStepDist = TIMELAPSE_STEP_DIST_MIN_MS;
  }
  if (newStepDist > TIMELAPSE_STEP_DIST_MAX_MS) {
    newStepDist = TIMELAPSE_STEP_DIST_MAX_MS;
  }
  if (newStepDist == stepDist) {
    startLimitReachedRumbleFeedback();
    Serial.println("Timelapse stepDist limit reached.");
    return;
  }

  stepDist = newStepDist;
  Serial.print("Timelapse stepDist (ms) = ");
  Serial.println(stepDist);
  startStepDistRumbleFeedback();
}

// START + PAD_UP increases timelapseIntervalSeconds.
// START + PAD_DOWN decreases timelapseIntervalSeconds.
// This is only active while no auto mode is running so it does not conflict
// with active timelapse or bounce motion.
bool handleTimelapseIntervalAdjustment() {
  bool adjustmentAllowed = (timelapseMode == 0 && bounce == 0);
  bool intervalAdjustUpComboActive = adjustmentAllowed && ps2x.Button(PSB_START) && ps2x.Button(PSB_PAD_UP);
  bool intervalAdjustDownComboActive = adjustmentAllowed && ps2x.Button(PSB_START) && ps2x.Button(PSB_PAD_DOWN);

  if (intervalAdjustUpComboActive && !lastIntervalAdjustUpComboActive) {
    adjustIntervalSeconds(1);
  }

  if (intervalAdjustDownComboActive && !lastIntervalAdjustDownComboActive) {
    adjustIntervalSeconds(-1);
  }

  lastIntervalAdjustUpComboActive = intervalAdjustUpComboActive;
  lastIntervalAdjustDownComboActive = intervalAdjustDownComboActive;

  if (intervalAdjustUpComboActive || intervalAdjustDownComboActive) {
    stopAllMotors();
    return true;
  }

  return false;
}

// SELECT + PAD_RIGHT increases stepDist.
// SELECT + PAD_LEFT decreases stepDist.
// stepDist changes by 10 ms per press.
// This is only active while no auto mode is running so it does not conflict
// with active timelapse or bounce motion.
bool handleTimelapseStepDistAdjustment() {
  bool adjustmentAllowed = (timelapseMode == 0 && bounce == 0);
  bool stepDistAdjustUpComboActive = adjustmentAllowed && ps2x.Button(PSB_SELECT) && ps2x.Button(PSB_PAD_RIGHT);
  bool stepDistAdjustDownComboActive = adjustmentAllowed && ps2x.Button(PSB_SELECT) && ps2x.Button(PSB_PAD_LEFT);

  if (stepDistAdjustUpComboActive && !lastStepDistAdjustUpComboActive) {
    adjustStepDist(TIMELAPSE_STEP_DIST_ADJUST_INCREMENT_MS);
  }

  if (stepDistAdjustDownComboActive && !lastStepDistAdjustDownComboActive) {
    adjustStepDist(-TIMELAPSE_STEP_DIST_ADJUST_INCREMENT_MS);
  }

  lastStepDistAdjustUpComboActive = stepDistAdjustUpComboActive;
  lastStepDistAdjustDownComboActive = stepDistAdjustDownComboActive;

  if (stepDistAdjustUpComboActive || stepDistAdjustDownComboActive) {
    stopAllMotors();
    return true;
  }

  return false;
}

void handleThumbstickCancel() {
  if (!ps2x.ButtonReleased(PSB_R3)) {
    return;
  }

  if (timelapseMode != 0) {
    resetTimelapseState();
    return;
  }

  if (bounce != 0) {
    resetBounceState();
    return;
  }

  if (intervalRumblePhase != INTERVAL_RUMBLE_IDLE || stepDistRumblePhase != STEP_DIST_RUMBLE_IDLE) {
    stopIntervalRumbleFeedback();
    Serial.println("Rumble feedback canceled.");
  }
}

void logEmergencyStopEvent() {
  Serial.print("EMERGENCY STOP | timelapseMode=");
  Serial.print(timelapseMode);
  Serial.print(" bounce=");
  Serial.print(bounce);
  Serial.print(" intervalSeconds=");
  Serial.print(timelapseIntervalSeconds);
  Serial.print(" stepDistMs=");
  Serial.println(stepDist);
}

// Emergency stop combo: L1 + L2 + R1 + R2
// Immediately stops all motor outputs and clears active auto modes.
bool handleEmergencyStop() {
  bool emergencyStopComboActive = ps2x.Button(PSB_L1) && ps2x.Button(PSB_L2) && ps2x.Button(PSB_R1) && ps2x.Button(PSB_R2);

  if (!emergencyStopComboActive) {
    lastEmergencyStopComboActive = false;
    return false;
  }

  if (!lastEmergencyStopComboActive) {
    logEmergencyStopEvent();
    resetTimelapseState();
    resetBounceState();
    stopIntervalRumbleFeedback();
  }

  stopAllMotors();
  lastEmergencyStopComboActive = true;
  return true;
}

const char* getTimelapseModeLabel(int mode) {
  switch (mode) {
    // Mode 1: swing left, boom down
    case 1:
      return "Timelapse Mode 1: Swing left, boom down";
    // Mode 2: swing left, boom up
    case 2:
      return "Timelapse Mode 2: Swing left, boom up";
    // Mode 3: swing right, boom up
    case 3:
      return "Timelapse Mode 3: Swing right, boom up";
    // Mode 4: swing right, boom down
    case 4:
      return "Timelapse Mode 4: Swing right, boom down";
    // Mode 5: swing left
    case 5:
      return "Timelapse Mode 5: Swing left";
    // Mode 6: boom up
    case 6:
      return "Timelapse Mode 6: Boom up";
    // Mode 7: swing right
    case 7:
      return "Timelapse Mode 7: Swing right";
    // Mode 8: boom down
    case 8:
      return "Timelapse Mode 8: Boom down";
    default:
      return nullptr;
  }
}

void applyTimelapseModeOutputs(int mode) {
  switch (mode) {
    // Mode 1: swing left, boom down
    case 1:
      setDirectionalOutput(isSwingReversed, swingLeft, swingRight, HIGH);
      setDirectionalOutput(isPanReversed, panRight, panLeft, HIGH);
      setDirectionalOutput(isLiftReversed, liftDown, liftUp, HIGH);
      setDirectionalOutput(isTiltReversed, tiltUp, tiltDown, HIGH);
      break;
    // Mode 2: swing left, boom up
    case 2:
      setDirectionalOutput(isSwingReversed, swingLeft, swingRight, HIGH);
      setDirectionalOutput(isPanReversed, panRight, panLeft, HIGH);
      setDirectionalOutput(isLiftReversed, liftUp, liftDown, HIGH);
      setDirectionalOutput(isTiltReversed, tiltDown, tiltUp, HIGH);
      break;
    // Mode 3: swing right, boom up
    case 3:
      setDirectionalOutput(isSwingReversed, swingRight, swingLeft, HIGH);
      setDirectionalOutput(isPanReversed, panLeft, panRight, HIGH);
      setDirectionalOutput(isLiftReversed, liftUp, liftDown, HIGH);
      setDirectionalOutput(isTiltReversed, tiltDown, tiltUp, HIGH);
      break;
    // Mode 4: swing right, boom down
    case 4:
      setDirectionalOutput(isSwingReversed, swingRight, swingLeft, HIGH);
      setDirectionalOutput(isPanReversed, panLeft, panRight, HIGH);
      setDirectionalOutput(isLiftReversed, liftDown, liftUp, HIGH);
      setDirectionalOutput(isTiltReversed, tiltUp, tiltDown, HIGH);
      break;
    // Mode 5: swing left
    case 5:
      setDirectionalOutput(isSwingReversed, swingLeft, swingRight, HIGH);
      setDirectionalOutput(isPanReversed, panRight, panLeft, HIGH);
      break;
    // Mode 6: boom up
    case 6:
      setDirectionalOutput(isLiftReversed, liftUp, liftDown, HIGH);
      setDirectionalOutput(isTiltReversed, tiltDown, tiltUp, HIGH);
      break;
    // Mode 7: swing right
    case 7:
      setDirectionalOutput(isSwingReversed, swingRight, swingLeft, HIGH);
      setDirectionalOutput(isPanReversed, panLeft, panRight, HIGH);
      break;
    // Mode 8: boom down
    case 8:
      setDirectionalOutput(isLiftReversed, liftDown, liftUp, HIGH);
      setDirectionalOutput(isTiltReversed, tiltUp, tiltDown, HIGH);
      break;
  }
}

void resetTimelapseState() {
  timelapseMode = 0;
  timelapsePhase = TIMELAPSE_PHASE_IDLE;
  timelapsePhaseStartMs = 0;
  digitalWrite(trigger, HIGH);
  stopAllMotors();
}

void resetBounceState() {
  bounce = 0;
  stage = 0;
  bouncePhaseStartMs = 0;
  bounceMoveDurationMs = 0;
  stopAllMotors();
}

// Returns 1-8 based on left-stick position, or 0 if no match.
// Used by both timelapse and bounce mode selection.
int stickPositionToMode(int stickX, int stickY) {
  if (stickX < STICK_MODE_LOW_THRESHOLD  && stickY > STICK_MODE_HIGH_THRESHOLD) return 1;
  if (stickX < STICK_MODE_LOW_THRESHOLD  && stickY < STICK_MODE_LOW_THRESHOLD)  return 2;
  if (stickX > STICK_MODE_HIGH_THRESHOLD && stickY < STICK_MODE_LOW_THRESHOLD)  return 3;
  if (stickX > STICK_MODE_HIGH_THRESHOLD && stickY > STICK_MODE_HIGH_THRESHOLD) return 4;
  if (stickX == STICK_MIN) return 5;
  if (stickY == STICK_MIN) return 6;
  if (stickX == STICK_MAX) return 7;
  if (stickY == STICK_MAX) return 8;
  return 0;
}

void updateTimelapseModeSelection() {
  if (timelapseMode != 0 || !ps2x.ButtonReleased(PSB_SELECT)) {
    return;
  }
  timelapseMode = stickPositionToMode(leftStickXvalue, leftStickYvalue);
}

const char* getBounceModeSerialLabel(int mode) {
  switch (mode) {
    case 1:
      return "bounce 1";
    case 2:
      return "bounce 2";
    case 3:
      return "bounce 3";
    case 4:
      return "bounce 4";
    case 5:
      return "bounce 5";
    case 6:
      return "bounce 6";
    case 7:
      return "bounce 7";
    case 8:
      return "bounce 8";
    default:
      return nullptr;
  }
}

void updateBounceModeSelection() {
  if (bounce != 0 || !ps2x.ButtonReleased(PSB_START)) {
    return;
  }
  bounce = stickPositionToMode(leftStickXvalue, leftStickYvalue);
  const char* bounceLabel = getBounceModeSerialLabel(bounce);
  if (bounceLabel != nullptr) {
    Serial.println(bounceLabel);
  }
}

// Drives the motors for the given bounce mode toward or away from the endpoint.
// towardEndpoint=true  → moving toward the target position (stage 0 travel + stage 1 second half)
// towardEndpoint=false → moving back toward the start position (stage 1 first half)
// state=HIGH turns the motor on; state=LOW turns it off.
// DIP-switch reversal is handled inside setDirectionalOutput().
void setBounceModeOutputs(int mode, bool towardEndpoint, uint8_t state) {
  switch (mode) {
    case 1: // swing left + boom down
      if (towardEndpoint) {
        setDirectionalOutput(isSwingReversed, swingLeft,  swingRight, state);
        setDirectionalOutput(isPanReversed,   panRight,   panLeft,    state);
        setDirectionalOutput(isLiftReversed,  liftDown,   liftUp,     state);
        setDirectionalOutput(isTiltReversed,  tiltUp,     tiltDown,   state);
      } else {
        setDirectionalOutput(isSwingReversed, swingRight, swingLeft,  state);
        setDirectionalOutput(isPanReversed,   panLeft,    panRight,   state);
        setDirectionalOutput(isLiftReversed,  liftUp,     liftDown,   state);
        setDirectionalOutput(isTiltReversed,  tiltDown,   tiltUp,     state);
      }
      break;
    case 2: // swing left + boom up
      if (towardEndpoint) {
        setDirectionalOutput(isSwingReversed, swingLeft,  swingRight, state);
        setDirectionalOutput(isPanReversed,   panRight,   panLeft,    state);
        setDirectionalOutput(isLiftReversed,  liftUp,     liftDown,   state);
        setDirectionalOutput(isTiltReversed,  tiltDown,   tiltUp,     state);
      } else {
        setDirectionalOutput(isSwingReversed, swingRight, swingLeft,  state);
        setDirectionalOutput(isPanReversed,   panLeft,    panRight,   state);
        setDirectionalOutput(isLiftReversed,  liftDown,   liftUp,     state);
        setDirectionalOutput(isTiltReversed,  tiltUp,     tiltDown,   state);
      }
      break;
    case 3: // swing right + boom up
      if (towardEndpoint) {
        setDirectionalOutput(isSwingReversed, swingRight, swingLeft,  state);
        setDirectionalOutput(isPanReversed,   panLeft,    panRight,   state);
        setDirectionalOutput(isLiftReversed,  liftUp,     liftDown,   state);
        setDirectionalOutput(isTiltReversed,  tiltDown,   tiltUp,     state);
      } else {
        setDirectionalOutput(isSwingReversed, swingLeft,  swingRight, state);
        setDirectionalOutput(isPanReversed,   panRight,   panLeft,    state);
        setDirectionalOutput(isLiftReversed,  liftDown,   liftUp,     state);
        setDirectionalOutput(isTiltReversed,  tiltUp,     tiltDown,   state);
      }
      break;
    case 4: // swing right + boom down
      if (towardEndpoint) {
        setDirectionalOutput(isSwingReversed, swingRight, swingLeft,  state);
        setDirectionalOutput(isPanReversed,   panLeft,    panRight,   state);
        setDirectionalOutput(isLiftReversed,  liftDown,   liftUp,     state);
        setDirectionalOutput(isTiltReversed,  tiltUp,     tiltDown,   state);
      } else {
        setDirectionalOutput(isSwingReversed, swingLeft,  swingRight, state);
        setDirectionalOutput(isPanReversed,   panRight,   panLeft,    state);
        setDirectionalOutput(isLiftReversed,  liftUp,     liftDown,   state);
        setDirectionalOutput(isTiltReversed,  tiltDown,   tiltUp,     state);
      }
      break;
    case 5: // swing left only
      if (towardEndpoint) {
        setDirectionalOutput(isSwingReversed, swingLeft,  swingRight, state);
        setDirectionalOutput(isPanReversed,   panRight,   panLeft,    state);
      } else {
        setDirectionalOutput(isSwingReversed, swingRight, swingLeft,  state);
        setDirectionalOutput(isPanReversed,   panLeft,    panRight,   state);
      }
      break;
    case 6: // boom up only
      if (towardEndpoint) {
        setDirectionalOutput(isLiftReversed,  liftUp,     liftDown,   state);
        setDirectionalOutput(isTiltReversed,  tiltDown,   tiltUp,     state);
      } else {
        setDirectionalOutput(isLiftReversed,  liftDown,   liftUp,     state);
        setDirectionalOutput(isTiltReversed,  tiltUp,     tiltDown,   state);
      }
      break;
    case 7: // swing right only
      if (towardEndpoint) {
        setDirectionalOutput(isSwingReversed, swingRight, swingLeft,  state);
        setDirectionalOutput(isPanReversed,   panLeft,    panRight,   state);
      } else {
        setDirectionalOutput(isSwingReversed, swingLeft,  swingRight, state);
        setDirectionalOutput(isPanReversed,   panRight,   panLeft,    state);
      }
      break;
    case 8: // boom down only
      if (towardEndpoint) {
        setDirectionalOutput(isLiftReversed,  liftDown,   liftUp,     state);
        setDirectionalOutput(isTiltReversed,  tiltUp,     tiltDown,   state);
      } else {
        setDirectionalOutput(isLiftReversed,  liftUp,     liftDown,   state);
        setDirectionalOutput(isTiltReversed,  tiltDown,   tiltUp,     state);
      }
      break;
  }
}

// Stops only the motors used by the given bounce mode.
// Called at the turnaround point between stage 0 and stage 1,
// and during direction switches inside stage 1.
// Modes 1-4 use all four axes; modes 5/7 use swing only; modes 6/8 use lift only.
void stopBounceModeOutputs(int mode) {
  switch (mode) {
    case 1: // swing left + boom down
    case 2: // swing left + boom up
    case 3: // swing right + boom up
    case 4: // swing right + boom down
      digitalWrite(swingLeft, LOW);
      digitalWrite(swingRight, LOW);
      digitalWrite(panLeft, LOW);
      digitalWrite(panRight, LOW);
      digitalWrite(liftUp, LOW);
      digitalWrite(liftDown, LOW);
      digitalWrite(tiltUp, LOW);
      digitalWrite(tiltDown, LOW);
      break;
    case 5: // swing left only
    case 7: // swing right only
      digitalWrite(swingLeft, LOW);
      digitalWrite(swingRight, LOW);
      digitalWrite(panLeft, LOW);
      digitalWrite(panRight, LOW);
      break;
    case 6: // boom up only
    case 8: // boom down only
      digitalWrite(liftUp, LOW);
      digitalWrite(liftDown, LOW);
      digitalWrite(tiltUp, LOW);
      digitalWrite(tiltDown, LOW);
      break;
  }
}

void advanceBounceToStage1() {
  unsigned long now = millis();
  stopBounceModeOutputs(bounce);
  bounceMoveDurationMs = now - bouncePhaseStartMs;
  bouncePhaseStartMs = now;
  stage = 1;
}

void handleBounceStage0(unsigned long now) {
  if (bounce == 0 || stage != 0) {
    return;
  }

  if (bouncePhaseStartMs == 0) {
    bouncePhaseStartMs = now;
  }

  setBounceModeOutputs(bounce, true, HIGH);

  if (ps2x.ButtonReleased(PSB_L3)) {
    advanceBounceToStage1();
  }
}

void handleBounceStage1(unsigned long now) {
  if (bounce == 0 || stage != 1) {
    return;
  }

  if (bounceMoveDurationMs == 0) {
    return;
  }

  unsigned long elapsed = (now - bouncePhaseStartMs) % (bounceMoveDurationMs * 2);

  if (elapsed < bounceMoveDurationMs) {
    // first half: traveling away from endpoint (back toward start)
    setBounceModeOutputs(bounce, true, LOW);
    setBounceModeOutputs(bounce, false, HIGH);
  } else {
    // second half: traveling toward endpoint
    setBounceModeOutputs(bounce, false, LOW);
    setBounceModeOutputs(bounce, true, HIGH);
  }
}

void handleActiveTimelapseMode(unsigned long now) {
  if (timelapseMode == 0) {
    return;
  }

  const char* modeLabel = getTimelapseModeLabel(timelapseMode);
  if (modeLabel == nullptr) {
    resetTimelapseState();
    return;
  }

  switch (timelapsePhase) {
    case TIMELAPSE_PHASE_IDLE:
      Serial.println(modeLabel);
      digitalWrite(trigger, LOW);
      timelapsePhase = TIMELAPSE_PHASE_TRIGGER_LOW;
      timelapsePhaseStartMs = now;
      break;
    case TIMELAPSE_PHASE_TRIGGER_LOW:
      if (now - timelapsePhaseStartMs >= static_cast<unsigned long>(timelapseIntervalMs / 2)) {
        digitalWrite(trigger, HIGH);
        timelapsePhase = TIMELAPSE_PHASE_TRIGGER_HIGH;
        timelapsePhaseStartMs = now;
      }
      break;
    case TIMELAPSE_PHASE_TRIGGER_HIGH:
      if (now - timelapsePhaseStartMs >= static_cast<unsigned long>(timelapseIntervalMs / 2)) {
        applyTimelapseModeOutputs(timelapseMode);
        timelapsePhase = TIMELAPSE_PHASE_MOVE_ACTIVE;
        timelapsePhaseStartMs = now;
      }
      break;
    case TIMELAPSE_PHASE_MOVE_ACTIVE:
      if (now - timelapsePhaseStartMs >= static_cast<unsigned long>(stepDist)) {
        stopAllMotors();
        timelapsePhase = TIMELAPSE_PHASE_IDLE;
        timelapsePhaseStartMs = now;
      }
      break;
    default:
      resetTimelapseState();
      return;
  }
}

void setup() {

  updateIntervalMs();

  const uint8_t outputPins[] = {
    swingLeft,
    swingRight,
    swingSpeedUp,
    swingSpeedDown,
    liftDown,
    liftUp,
    liftSpeedUp,
    liftSpeedDown,
    panLeft,
    panRight,
    panSpeedUp,
    panSpeedDown,
    tiltDown,
    tiltUp,
    tiltSpeedUp,
    tiltSpeedDown,
    focusLeft,
    focusRight,
    focusSpeedUp,
    focusSpeedDown,
    tiltSpeedUpOnly,
    tiltSpeedDownOnly,
    panSpeedUpOnly,
    panSpeedDownOnly,
    trigger
  };

  const uint8_t dipSwitchPins[] = {
    DIP_SWITCH_1,
    DIP_SWITCH_2,
    DIP_SWITCH_3,
    DIP_SWITCH_4,
    DIP_SWITCH_5
  };

  for (size_t i = 0; i < sizeof(outputPins) / sizeof(outputPins[0]); ++i) {
    pinMode(outputPins[i], OUTPUT);
  }

  for (size_t i = 0; i < sizeof(dipSwitchPins) / sizeof(dipSwitchPins[0]); ++i) {
    pinMode(dipSwitchPins[i], INPUT_PULLUP);
  }

  resetTimelapseState();

  delay(CONTROLLER_STARTUP_DELAY_MS);
  configureController();
  detectControllerType();
  Serial.println("START + PAD_UP/DOWN adjusts timelapseIntervalSeconds.");
  Serial.println("SELECT + PAD_RIGHT/LEFT adjusts stepDist by 10 ms.");
}

void loop() {

  if (error != 0) // skip loop on controller config error
    return;

  if (controllerType == 2) // skip Guitar Hero controller
    return;

  const bool isDualShockType = (controllerType == 1 || controllerType == 3);
  if (!isDualShockType) // skip unsupported controller types
    return;

  handleIntervalRumbleFeedback(millis());
  ps2x.read_gamepad(false, vibrate); // rumble intensity comes from the interval feedback state machine

  unsigned long now = millis();

  // Read DIP-switch reversal settings
  isSwingReversed = (digitalRead(DIP_SWITCH_1) == HIGH);
  isPanReversed = (digitalRead(DIP_SWITCH_2) == HIGH);
  isLiftReversed = (digitalRead(DIP_SWITCH_3) == HIGH);
  isTiltReversed = (digitalRead(DIP_SWITCH_4) == HIGH);
  isFocusReversed = (digitalRead(DIP_SWITCH_5) == HIGH);

  // Read analog stick values
  rightStickYvalue = ps2x.Analog(PSS_RY);
  rightStickXvalue = ps2x.Analog(PSS_RX);
  leftStickYvalue  = ps2x.Analog(PSS_LY);
  leftStickXvalue  = ps2x.Analog(PSS_LX);

  if (handleEmergencyStop()) {
    return;
  }

  handleThumbstickCancel();

  if (handleTimelapseIntervalAdjustment()) {
    return;
  }

  if (handleTimelapseStepDistAdjustment()) {
    return;
  }

  // Lift and Tilt speed control (R1/R2 buttons)
  handleAxisSpeedControl(PSB_R1, liftSpeedUp, tiltSpeedUp);
  handleAxisSpeedControl(PSB_R2, liftSpeedDown, tiltSpeedDown);

  // Pan and Swing speed control (L1/L2 buttons)
  handleAxisSpeedControl(PSB_L1, panSpeedUp, swingSpeedUp);
  handleAxisSpeedControl(PSB_L2, panSpeedDown, swingSpeedDown);

  // 1st AXIS (BOOM SWING)
  // swingLeft (no pan)
  handleSwingOnly(PSB_PAD_LEFT, swingLeft, swingRight);

  // swingRight (no pan)
  handleSwingOnly(PSB_PAD_RIGHT, swingRight, swingLeft);

  // swingLeft + panRight
  handleSwingAndPan(PSB_PAD_LEFT, swingLeft, swingRight, panRight, panLeft);

  // swingRight + panLeft
  handleSwingAndPan(PSB_PAD_RIGHT, swingRight, swingLeft, panLeft, panRight);

  handlePanTrimAxis();

  // 2nd AXIS (CAMERA PAN)
  handlePanAxis();

  // 3rd AXIS (BOOM LIFT)
  // lift UP (no tilt)
  handleLiftOnly(PSB_PAD_UP, liftUp, liftDown);

  // lift DOWN (no tilt)
  handleLiftOnly(PSB_PAD_DOWN, liftDown, liftUp);

  // lift UP + tilt DOWN
  handleLiftAndTilt(PSB_PAD_UP, liftUp, liftDown, tiltDown, tiltUp);

  // lift DOWN + tilt UP
  handleLiftAndTilt(PSB_PAD_DOWN, liftDown, liftUp, tiltUp, tiltDown);

  handleTiltTrimAxis();

  // 4th AXIS (CAMERA TILT)
  handleTiltAxis();

  // 5th AXIS (CAMERA FOCUS)
  handleFocusAxis();

  // Timelapse

  updateTimelapseModeSelection();
  handleActiveTimelapseMode(now);

  // MOCO Moves (bounce)

  updateBounceModeSelection();

  handleBounceStage0(now);

  handleBounceStage1(now);
}

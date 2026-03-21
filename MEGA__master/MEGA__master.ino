#include <PS2X_lib.h>  //for v1.6

constexpr uint8_t PS2_DAT = 10;
constexpr uint8_t PS2_CMD = 9;
constexpr uint8_t PS2_SEL = 8;
constexpr uint8_t PS2_CLK = 11;

constexpr bool PRESSURES = false;
constexpr bool RUMBLE = false;
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
int intervalSeconds = 15;
int interval;
int stepDist = 100;
const uint8_t trigger = 28;

// Motion Control Variables
int bounce = 0;
int count = 0;
int mocoDistance = 0;
int stage = 0;

constexpr uint8_t DIP_SWITCH_1 = 35;
constexpr uint8_t DIP_SWITCH_2 = 43;
constexpr uint8_t DIP_SWITCH_3 = 37;
constexpr uint8_t DIP_SWITCH_4 = 39;
constexpr uint8_t DIP_SWITCH_5 = 41;
constexpr unsigned long CONTROLLER_STARTUP_DELAY_MS = 300;
constexpr int STICK_MIN = 0;
constexpr int STICK_CENTER = 128;
constexpr int STICK_MAX = 255;
constexpr int TIMELAPSE_STICK_LOW_THRESHOLD = 123;
constexpr int TIMELAPSE_STICK_HIGH_THRESHOLD = 133;
constexpr int PAN_STOP_NONE = 0;
constexpr int PAN_STOP_ACTIVE = 1;
constexpr int PAN_STOP_TRIM = 2;
constexpr int TILT_STOP_NONE = 0;
constexpr int TILT_STOP_ACTIVE = 1;
constexpr int TILT_STOP_TRIM = 2;

bool isSwingReversed = false;
bool isPanReversed = false;
bool isLiftReversed = false;
bool isTiltReversed = false;
bool isFocusReversed = false;

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

void pulseTimelapseTrigger() {
  digitalWrite(trigger, LOW);
  delay(interval / 2);
  digitalWrite(trigger, HIGH);
  delay(interval / 2);
}

void updateTimelapseModeSelection() {
  if (timelapseMode != 0 || !ps2x.ButtonReleased(PSB_SELECT)) {
    return;
  }

  if (leftStickXvalue < TIMELAPSE_STICK_LOW_THRESHOLD && leftStickYvalue > TIMELAPSE_STICK_HIGH_THRESHOLD) {
    timelapseMode = 1;
  } else if (leftStickXvalue < TIMELAPSE_STICK_LOW_THRESHOLD && leftStickYvalue < TIMELAPSE_STICK_LOW_THRESHOLD) {
    timelapseMode = 2;
  } else if (leftStickXvalue > TIMELAPSE_STICK_HIGH_THRESHOLD && leftStickYvalue < TIMELAPSE_STICK_LOW_THRESHOLD) {
    timelapseMode = 3;
  } else if (leftStickXvalue > TIMELAPSE_STICK_HIGH_THRESHOLD && leftStickYvalue > TIMELAPSE_STICK_HIGH_THRESHOLD) {
    timelapseMode = 4;
  } else if (leftStickXvalue == STICK_MIN) {
    timelapseMode = 5;
  } else if (leftStickYvalue == STICK_MIN) {
    timelapseMode = 6;
  } else if (leftStickXvalue == STICK_MAX) {
    timelapseMode = 7;
  } else if (leftStickYvalue == STICK_MAX) {
    timelapseMode = 8;
  }
}

void handleActiveTimelapseMode() {
  if (timelapseMode == 0) {
    return;
  }

  switch (timelapseMode) {
    // Mode 1: swing left, boom down
    case 1:
      Serial.println("Timelapse Mode 1: Swing left, boom down");
      break;
    // Mode 2: swing left, boom up
    case 2:
      Serial.println("Timelapse Mode 2: Swing left, boom up");
      break;
    // Mode 3: swing right, boom up
    case 3:
      Serial.println("Timelapse Mode 3: Swing right, boom up");
      break;
    // Mode 4: swing right, boom down
    case 4:
      Serial.println("Timelapse Mode 4: Swing right, boom down");
      break;
    // Mode 5: swing left
    case 5:
      Serial.println("Timelapse Mode 5: Swing left");
      break;
    // Mode 6: boom up
    case 6:
      Serial.println("Timelapse Mode 6: Boom up");
      break;
    // Mode 7: swing right
    case 7:
      Serial.println("Timelapse Mode 7: Swing right");
      break;
    // Mode 8: boom down
    case 8:
      Serial.println("Timelapse Mode 8: Boom down");
      break;
    default:
      return;
  }

  pulseTimelapseTrigger();

  switch (timelapseMode) {
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
    default:
      return;
  }

  delay(stepDist);
  stopAllMotors();
}

void setup() {

  interval = intervalSeconds * 1000;

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

  delay(CONTROLLER_STARTUP_DELAY_MS);
  configureController();
  detectControllerType();
}

void loop() {

  if (error != 0) // skip loop on controller config error
    return;

  if (controllerType == 2) // skip Guitar Hero controller
    return;

  const bool isDualShockType = (controllerType == 1 || controllerType == 3);
  if (!isDualShockType) // skip unsupported controller types
    return;

  ps2x.read_gamepad(false, vibrate); // vibration disabled

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

  // R3 (right stick click) cancels active timelapse and stops all motors
  if (timelapseMode != 0 && ps2x.ButtonReleased(PSB_R3)) {
    timelapseMode = 0;
    stopAllMotors();
  }

  updateTimelapseModeSelection();
  handleActiveTimelapseMode();

  // MOCO Moves (bounce)

  // R3 (right stick click) cancels active bounce and stops all motors
  if (bounce != 0 && ps2x.ButtonReleased(PSB_R3)) {
    bounce = 0;
    stage = 0;
    count = 0;
    stopAllMotors();
  }

  // Bounce 1: swing left, boom down
  if (bounce == 0 && leftStickXvalue < 123 && leftStickYvalue > 133 && ps2x.ButtonReleased(PSB_START)) {
    bounce = 1;
    Serial.println ("bounce 1");
  }

  if (bounce == 1 && stage == 0) {
    if (!isSwingReversed) {
      digitalWrite(swingLeft, HIGH);
    }
    if (isSwingReversed) {
      digitalWrite(swingRight, HIGH);
    }
    if (!isPanReversed) {
      digitalWrite(panRight, HIGH);
    }
    if (isPanReversed) {
      digitalWrite(panLeft, HIGH);
    }

    if (!isLiftReversed) {
      digitalWrite(liftDown, HIGH);
    }
    if (isLiftReversed) {
      digitalWrite(liftUp, HIGH);
    }

    if (!isTiltReversed) {
      digitalWrite(tiltUp, HIGH);
    }
    if (isTiltReversed) {
      digitalWrite(tiltDown, HIGH);
    }
    count++;

  }
  if (bounce == 1 && stage == 0 && ps2x.ButtonReleased(PSB_L3)) {

    digitalWrite(swingLeft, LOW);
    digitalWrite(swingRight, LOW);
    digitalWrite(panRight, LOW);
    digitalWrite(panLeft, LOW);
    digitalWrite(liftDown, LOW);
    digitalWrite(liftUp, LOW);
    digitalWrite(tiltUp, LOW);
    digitalWrite(tiltDown, LOW);

    mocoDistance = count;
    count = 0;
    stage = 1;
  }

  if (bounce == 1 && stage == 1) { //end point
    if (count <= mocoDistance) {

      //turn off motors
      if (!isSwingReversed) {
        digitalWrite(swingLeft, LOW);
      }
      if (isSwingReversed) {
        digitalWrite(swingRight, LOW);
      }
      if (!isPanReversed) {
        digitalWrite(panRight, LOW);
      }
      if (isPanReversed) {
        digitalWrite(panLeft, LOW);
      }

      if (!isLiftReversed) {
        digitalWrite(liftDown, LOW);
      }
      if (isLiftReversed) {
        digitalWrite(liftUp, LOW);
      }

      if (!isTiltReversed) {
        digitalWrite(tiltUp, LOW);
      }
      if (isTiltReversed) {
        digitalWrite(tiltDown, LOW);
      }

      //turn on motors

      if (!isSwingReversed) {
        digitalWrite(swingRight, HIGH);
      }
      if (isSwingReversed) {
        digitalWrite(swingLeft, HIGH);
      }
      if (!isPanReversed) {
        digitalWrite(panLeft, HIGH);
      }
      if (isPanReversed) {
        digitalWrite(panRight, HIGH);
      }

      if (!isLiftReversed) {
        digitalWrite(liftUp, HIGH);
      }
      if (isLiftReversed) {
        digitalWrite(liftDown, HIGH);
      }

      if (!isTiltReversed) {
        digitalWrite(tiltDown, HIGH);
      }
      if (isTiltReversed) {
        digitalWrite(tiltUp, HIGH);
      }
      count++;
    }

    if (count >= mocoDistance) {//starting point

      if (!isSwingReversed) {
        digitalWrite(swingRight, LOW);
      }
      if (isSwingReversed) {
        digitalWrite(swingLeft, LOW);
      }
      if (!isPanReversed) {
        digitalWrite(panLeft, LOW);
      }
      if (isPanReversed) {
        digitalWrite(panRight, LOW);
      }

      if (!isLiftReversed) {
        digitalWrite(liftUp, LOW);
      }
      if (isLiftReversed) {
        digitalWrite(liftDown, LOW);
      }

      if (!isTiltReversed) {
        digitalWrite(tiltDown, LOW);
      }
      if (isTiltReversed) {
        digitalWrite(tiltUp, LOW);
      }

      //turn motors on

      if (!isSwingReversed) {
        digitalWrite(swingLeft, HIGH);
      }
      if (isSwingReversed) {
        digitalWrite(swingRight, HIGH);
      }
      if (!isPanReversed) {
        digitalWrite(panRight, HIGH);
      }
      if (isPanReversed) {
        digitalWrite(panLeft, HIGH);
      }

      if (!isLiftReversed) {
        digitalWrite(liftDown, HIGH);
      }
      if (isLiftReversed) {
        digitalWrite(liftUp, HIGH);
      }

      if (!isTiltReversed) {
        digitalWrite(tiltUp, HIGH);
      }
      if (isTiltReversed) {
        digitalWrite(tiltDown, HIGH);
      }
      count++;
    }

    if (count >= mocoDistance * 2) {
      count = 0;
    }
  }

  // Bounce 2: swing left, boom up
  if (bounce == 0 && leftStickXvalue < 123 && leftStickYvalue < 123 && ps2x.ButtonReleased(PSB_START))  {
    bounce = 2;
    Serial.println ("bounce 2");
  }
  if (bounce == 2 && stage == 0) {
    if (!isSwingReversed) {
      digitalWrite(swingLeft, HIGH);
    }
    if (isSwingReversed) {
      digitalWrite(swingRight, HIGH);
    }
    if (!isPanReversed) {
      digitalWrite(panRight, HIGH);
    }
    if (isPanReversed) {
      digitalWrite(panLeft, HIGH);
    }

    if (!isLiftReversed) {
      digitalWrite(liftUp, HIGH);
    }
    if (isLiftReversed) {
      digitalWrite(liftDown, HIGH);
    }

    if (!isTiltReversed) {
      digitalWrite(tiltDown, HIGH);
    }
    if (isTiltReversed) {
      digitalWrite(tiltUp, HIGH);
    }
    count++;

  }
  if (bounce == 2 && stage == 0 && ps2x.ButtonReleased(PSB_L3)) {
    digitalWrite(swingLeft, LOW);
    digitalWrite(swingRight, LOW);
    digitalWrite(panRight, LOW);
    digitalWrite(panLeft, LOW);
    digitalWrite(liftDown, LOW);
    digitalWrite(liftUp, LOW);
    digitalWrite(tiltUp, LOW);
    digitalWrite(tiltDown, LOW);

    mocoDistance = count;
    count = 0;
    stage = 1;
  }

  if (bounce == 2 && stage == 1) {
    if (count <= mocoDistance) {

      //turn motors off

      if (!isSwingReversed) {
        digitalWrite(swingLeft, LOW);
      }
      if (isSwingReversed) {
        digitalWrite(swingRight, LOW);
      }
      if (!isPanReversed) {
        digitalWrite(panRight, LOW);
      }
      if (isPanReversed) {
        digitalWrite(panLeft, LOW);
      }

      if (!isLiftReversed) {
        digitalWrite(liftUp, LOW);
      }
      if (isLiftReversed) {
        digitalWrite(liftDown, LOW);
      }

      if (!isTiltReversed) {
        digitalWrite(tiltDown, LOW);
      }
      if (isTiltReversed) {
        digitalWrite(tiltUp, LOW);
      }

      //turn motors on

      if (!isSwingReversed) {
        digitalWrite(swingRight, HIGH);
      }
      if (isSwingReversed) {
        digitalWrite(swingLeft, HIGH);
      }
      if (!isPanReversed) {
        digitalWrite(panLeft, HIGH);
      }
      if (isPanReversed) {
        digitalWrite(panRight, HIGH);
      }

      if (!isLiftReversed) {
        digitalWrite(liftDown, HIGH);
      }
      if (isLiftReversed) {
        digitalWrite(liftUp, HIGH);
      }

      if (!isTiltReversed) {
        digitalWrite(tiltUp, HIGH);
      }
      if (isTiltReversed) {
        digitalWrite(tiltDown, HIGH);
      }
      count++;
    }

    if (count >= mocoDistance) {

      //turn motors off
      if (!isSwingReversed) {
        digitalWrite(swingRight, LOW);
      }
      if (isSwingReversed) {
        digitalWrite(swingLeft, LOW);
      }
      if (!isPanReversed) {
        digitalWrite(panLeft, LOW);
      }
      if (isPanReversed) {
        digitalWrite(panRight, LOW);
      }

      if (!isLiftReversed) {
        digitalWrite(liftDown, LOW);
      }
      if (isLiftReversed) {
        digitalWrite(liftUp, LOW);
      }

      if (!isTiltReversed) {
        digitalWrite(tiltUp, LOW);
      }
      if (isTiltReversed) {
        digitalWrite(tiltDown, LOW);
      }

      //turn motors on
      if (!isSwingReversed) {
        digitalWrite(swingLeft, HIGH);
      }
      if (isSwingReversed) {
        digitalWrite(swingRight, HIGH);
      }
      if (!isPanReversed) {
        digitalWrite(panRight, HIGH);
      }
      if (isPanReversed) {
        digitalWrite(panLeft, HIGH);
      }

      if (!isLiftReversed) {
        digitalWrite(liftUp, HIGH);
      }
      if (isLiftReversed) {
        digitalWrite(liftDown, HIGH);
      }

      if (!isTiltReversed) {
        digitalWrite(tiltDown, HIGH);
      }
      if (isTiltReversed) {
        digitalWrite(tiltUp, HIGH);
      }
      count++;
    }

    if (count >= mocoDistance * 2) {
      count = 0;
    }
  }

  // Bounce 3: swing right, boom up
  if (bounce == 0 && leftStickXvalue > 133 && leftStickYvalue < 123 && ps2x.ButtonReleased(PSB_START)) {
    bounce = 3; //swing right boom up
    Serial.println ("bounce 3");
  }

  if (bounce == 3 && stage == 0) {

    if (!isSwingReversed) {
      digitalWrite(swingRight, HIGH);
    }
    if (isSwingReversed) {
      digitalWrite(swingLeft, HIGH);
    }
    if (!isPanReversed) {
      digitalWrite(panLeft, HIGH);
    }
    if (isPanReversed) {
      digitalWrite(panRight, HIGH);
    }

    if (!isLiftReversed) {
      digitalWrite(liftUp, HIGH);
    }
    if (isLiftReversed) {
      digitalWrite(liftDown, HIGH);
    }

    if (!isTiltReversed) {
      digitalWrite(tiltDown, HIGH);
    }
    if (isTiltReversed) {
      digitalWrite(tiltUp, HIGH);
    }

    count++;
  }

  if (bounce == 3 && stage == 0 && ps2x.ButtonReleased(PSB_L3)) {

    digitalWrite(swingLeft, LOW);
    digitalWrite(swingRight, LOW);
    digitalWrite(panRight, LOW);
    digitalWrite(panLeft, LOW);
    digitalWrite(liftDown, LOW);
    digitalWrite(liftUp, LOW);
    digitalWrite(tiltUp, LOW);
    digitalWrite(tiltDown, LOW);

    mocoDistance = count;
    count = 0;
    stage = 1;
  }

  if (bounce == 3 && stage == 1) {

    if (count <= mocoDistance) {

      // motor off
      if (!isSwingReversed) {
        digitalWrite(swingRight, LOW);
      }
      if (isSwingReversed) {
        digitalWrite(swingLeft, LOW);
      }
      if (!isPanReversed) {
        digitalWrite(panLeft, LOW);
      }
      if (isPanReversed) {
        digitalWrite(panRight, LOW);
      }

      if (!isLiftReversed) {
        digitalWrite(liftUp, LOW);
      }
      if (isLiftReversed) {
        digitalWrite(liftDown, LOW);
      }

      if (!isTiltReversed) {
        digitalWrite(tiltDown, LOW);
      }
      if (isTiltReversed) {
        digitalWrite(tiltUp, LOW);
      }

      //motor on
      if (!isSwingReversed) {
        digitalWrite(swingLeft, HIGH);
      }
      if (isSwingReversed) {
        digitalWrite(swingRight, HIGH);
      }
      if (!isPanReversed) {
        digitalWrite(panRight, HIGH);
      }
      if (isPanReversed) {
        digitalWrite(panLeft, HIGH);
      }

      if (!isLiftReversed) {
        digitalWrite(liftDown, HIGH);
      }
      if (isLiftReversed) {
        digitalWrite(liftUp, HIGH);
      }

      if (!isTiltReversed) {
        digitalWrite(tiltUp, HIGH);
      }
      if (isTiltReversed) {
        digitalWrite(tiltDown, HIGH);
      }

      count++;
    }

    if (count >= mocoDistance) {

      //motor off
      if (!isSwingReversed) {
        digitalWrite(swingLeft, LOW);
      }
      if (isSwingReversed) {
        digitalWrite(swingRight, LOW);
      }
      if (!isPanReversed) {
        digitalWrite(panRight, LOW);
      }
      if (isPanReversed) {
        digitalWrite(panLeft, LOW);
      }

      if (!isLiftReversed) {
        digitalWrite(liftDown, LOW);
      }
      if (isLiftReversed) {
        digitalWrite(liftUp, LOW);
      }

      if (!isTiltReversed) {
        digitalWrite(tiltUp, LOW);
      }
      if (isTiltReversed) {
        digitalWrite(tiltDown, LOW);
      }

      //motor on
      if (!isSwingReversed) {
        digitalWrite(swingRight, HIGH);
      }
      if (isSwingReversed) {
        digitalWrite(swingLeft, HIGH);
      }
      if (!isPanReversed) {
        digitalWrite(panLeft, HIGH);
      }
      if (isPanReversed) {
        digitalWrite(panRight, HIGH);
      }

      if (!isLiftReversed) {
        digitalWrite(liftUp, HIGH);
      }
      if (isLiftReversed) {
        digitalWrite(liftDown, HIGH);
      }

      if (!isTiltReversed) {
        digitalWrite(tiltDown, HIGH);
      }
      if (isTiltReversed) {
        digitalWrite(tiltUp, HIGH);
      }

      count++;
    }
    if (count >= mocoDistance * 2) {
      count = 0;
    }
  }

  // Bounce 4: swing right, boom down
  if (bounce == 0 && leftStickXvalue > 133 && leftStickYvalue > 133 && ps2x.ButtonReleased(PSB_START)) {
    bounce = 4;
    Serial.println ("bounce 4");
  }

  if (bounce == 4 && stage == 0) {

    if (!isSwingReversed) {
      digitalWrite(swingRight, HIGH);
    }
    if (isSwingReversed) {
      digitalWrite(swingLeft, HIGH);
    }
    if (!isPanReversed) {
      digitalWrite(panLeft, HIGH);
    }
    if (isPanReversed) {
      digitalWrite(panRight, HIGH);
    }

    if (!isLiftReversed) {
      digitalWrite(liftDown, HIGH);
    }
    if (isLiftReversed) {
      digitalWrite(liftUp, HIGH);
    }

    if (!isTiltReversed) {
      digitalWrite(tiltUp, HIGH);
    }
    if (isTiltReversed) {
      digitalWrite(tiltDown, HIGH);
    }

    count++;

  }

  if (bounce == 4 && stage == 0 && ps2x.ButtonReleased(PSB_L3)) {

    digitalWrite(swingLeft, LOW);
    digitalWrite(swingRight, LOW);
    digitalWrite(panRight, LOW);
    digitalWrite(panLeft, LOW);
    digitalWrite(liftDown, LOW);
    digitalWrite(liftUp, LOW);
    digitalWrite(tiltUp, LOW);
    digitalWrite(tiltDown, LOW);
    mocoDistance = count;
    count = 0;
    stage = 1;

  }

  if (bounce == 4 && stage == 1) {
    if (count <= mocoDistance) {

      //motor off
      if (!isSwingReversed) {
        digitalWrite(swingRight, LOW);
      }
      if (isSwingReversed) {
        digitalWrite(swingLeft, LOW);
      }
      if (!isPanReversed) {
        digitalWrite(panLeft, LOW);
      }
      if (isPanReversed) {
        digitalWrite(panRight, LOW);
      }

      if (!isLiftReversed) {
        digitalWrite(liftDown, LOW);
      }
      if (isLiftReversed) {
        digitalWrite(liftUp, LOW);
      }

      if (!isTiltReversed) {
        digitalWrite(tiltUp, LOW);
      }
      if (isTiltReversed) {
        digitalWrite(tiltDown, LOW);
      }

      //motor on
      if (!isSwingReversed) {
        digitalWrite(swingLeft, HIGH);
      }
      if (isSwingReversed) {
        digitalWrite(swingRight, HIGH);
      }
      if (!isPanReversed) {
        digitalWrite(panRight, HIGH);
      }
      if (isPanReversed) {
        digitalWrite(panLeft, HIGH);
      }

      if (!isLiftReversed) {
        digitalWrite(liftUp, HIGH);
      }
      if (isLiftReversed) {
        digitalWrite(liftDown, HIGH);
      }

      if (!isTiltReversed) {
        digitalWrite(tiltDown, HIGH);
      }
      if (isTiltReversed) {
        digitalWrite(tiltUp, HIGH);
      }

      count++;
    }

    if (count >= mocoDistance) {

      //motor OFF
      if (!isSwingReversed) {
        digitalWrite(swingLeft, LOW);
      }
      if (isSwingReversed) {
        digitalWrite(swingRight, LOW);
      }
      if (!isPanReversed) {
        digitalWrite(panRight, LOW);
      }
      if (isPanReversed) {
        digitalWrite(panLeft, LOW);
      }

      if (!isLiftReversed) {
        digitalWrite(liftUp, LOW);
      }
      if (isLiftReversed) {
        digitalWrite(liftDown, LOW);
      }

      if (!isTiltReversed) {
        digitalWrite(tiltDown, LOW);
      }
      if (isTiltReversed) {
        digitalWrite(tiltUp, LOW);
      }

      //motor ON
      if (!isSwingReversed) {
        digitalWrite(swingRight, HIGH);
      }
      if (isSwingReversed) {
        digitalWrite(swingLeft, HIGH);
      }
      if (!isPanReversed) {
        digitalWrite(panLeft, HIGH);
      }
      if (isPanReversed) {
        digitalWrite(panRight, HIGH);
      }

      if (!isLiftReversed) {
        digitalWrite(liftDown, HIGH);
      }
      if (isLiftReversed) {
        digitalWrite(liftUp, HIGH);
      }

      if (!isTiltReversed) {
        digitalWrite(tiltUp, HIGH);
      }
      if (isTiltReversed) {
        digitalWrite(tiltDown, HIGH);
      }

      count++;
    }

    if (count >= mocoDistance * 2) {
      count = 0;
    }

  }

  // Bounce 5: swing left
  if (bounce == 0 && leftStickXvalue == 0 && ps2x.ButtonReleased(PSB_START)) {
    bounce = 5;// Swing left
    Serial.println ("bounce 5");
  }

  if (bounce == 5 && stage == 0) {

    if (!isSwingReversed) {
      digitalWrite(swingLeft, HIGH);
    }
    if (isSwingReversed) {
      digitalWrite(swingRight, HIGH);
    }
    if (!isPanReversed) {
      digitalWrite(panRight, HIGH);
    }
    if (isPanReversed) {
      digitalWrite(panLeft, HIGH);
    }
    count++;

  }
  if (bounce == 5 && stage == 0 && ps2x.ButtonReleased(PSB_L3)) {

    digitalWrite(swingLeft, LOW);
    digitalWrite(panRight, LOW);
    digitalWrite(swingRight, LOW);
    digitalWrite(panLeft, LOW);

    mocoDistance = count;
    count = 0;
    stage = 1;
  }

  if (bounce == 5 && stage == 1) {
    if (count <= mocoDistance) {

      //motor OFF
      if (!isSwingReversed) {
        digitalWrite(swingLeft, LOW);
      }
      if (isSwingReversed) {
        digitalWrite(swingRight, LOW);
      }
      if (!isPanReversed) {
        digitalWrite(panRight, LOW);
      }
      if (isPanReversed) {
        digitalWrite(panLeft, LOW);
      }
      //motor ON
      if (!isSwingReversed) {
        digitalWrite(swingRight, HIGH);
      }
      if (isSwingReversed) {
        digitalWrite(swingLeft, HIGH);
      }
      if (!isPanReversed) {
        digitalWrite(panLeft, HIGH);
      }
      if (isPanReversed) {
        digitalWrite(panRight, HIGH);
      }

      count++;
    }

    if (count >= mocoDistance) {

      //motor OFF
      if (!isSwingReversed) {
        digitalWrite(swingRight, LOW);
      }
      if (isSwingReversed) {
        digitalWrite(swingLeft, LOW);
      }
      if (!isPanReversed) {
        digitalWrite(panLeft, LOW);
      }
      if (isPanReversed) {
        digitalWrite(panRight, LOW);
      }

      //motor ON
      if (!isSwingReversed) {
        digitalWrite(swingLeft, HIGH);
      }
      if (isSwingReversed) {
        digitalWrite(swingRight, HIGH);
      }
      if (!isPanReversed) {
        digitalWrite(panRight, HIGH);
      }
      if (isPanReversed) {
        digitalWrite(panLeft, HIGH);
      }

      count++;
    }

    if (count >= mocoDistance * 2) {
      count = 0;
    }
  }

  // Bounce 6: boom up
  if (bounce == 0 && leftStickYvalue == 0 && ps2x.ButtonReleased(PSB_START))  {
    bounce = 6;   // boom up
    Serial.println ("bounce 6");
  }
  if (bounce == 6 && stage == 0) {

    if (!isLiftReversed) {
      digitalWrite(liftUp, HIGH);
    }
    if (isLiftReversed) {
      digitalWrite(liftDown, HIGH);
    }

    if (!isTiltReversed) {
      digitalWrite(tiltDown, HIGH);
    }
    if (isTiltReversed) {
      digitalWrite(tiltUp, HIGH);
    }
    count++;

  }
  if (bounce == 6 && stage == 0 && ps2x.ButtonReleased(PSB_L3)) {

    digitalWrite(liftUp, LOW);
    digitalWrite(tiltDown, LOW);
    digitalWrite(liftDown, LOW);
    digitalWrite(tiltUp, LOW);

    mocoDistance = count;
    count = 0;
    stage = 1;
  }

  if (bounce == 6 && stage == 1) {
    if (count <= mocoDistance) {

      if (!isLiftReversed) {
        digitalWrite(liftUp, LOW);
      }
      if (isLiftReversed) {
        digitalWrite(liftDown, LOW);
      }

      if (!isTiltReversed) {
        digitalWrite(tiltDown, LOW);
      }
      if (isTiltReversed) {
        digitalWrite(tiltUp, LOW);
      }

      if (!isLiftReversed) {
        digitalWrite(liftDown, HIGH);
      }
      if (isLiftReversed) {
        digitalWrite(liftUp, HIGH);
      }

      if (!isTiltReversed) {
        digitalWrite(tiltUp, HIGH);
      }
      if (isTiltReversed) {
        digitalWrite(tiltDown, HIGH);
      }

      count++;
    }

    if (count >= mocoDistance) {

      if (!isLiftReversed) {
        digitalWrite(liftDown, LOW);
      }
      if (isLiftReversed) {
        digitalWrite(liftUp, LOW);
      }

      if (!isTiltReversed) {
        digitalWrite(tiltUp, LOW);
      }
      if (isTiltReversed) {
        digitalWrite(tiltDown, LOW);
      }

      if (!isLiftReversed) {
        digitalWrite(liftUp, HIGH);
      }
      if (isLiftReversed) {
        digitalWrite(liftDown, HIGH);
      }

      if (!isTiltReversed) {
        digitalWrite(tiltDown, HIGH);
      }
      if (isTiltReversed) {
        digitalWrite(tiltUp, HIGH);
      }
      count++;
    }

    if (count >= mocoDistance * 2) {
      count = 0;
    }
  }

  // Bounce 7: swing right
  if (bounce == 0 && leftStickXvalue == 255 && ps2x.ButtonReleased(PSB_START)) {
    bounce = 7;
    Serial.println ("bounce 7");
  }

  if (bounce == 7 && stage == 0) {
    if (!isSwingReversed) {
      digitalWrite(swingRight, HIGH);
    }
    if (isSwingReversed) {
      digitalWrite(swingLeft, HIGH);
    }
    if (!isPanReversed) {
      digitalWrite(panLeft, HIGH);
    }
    if (isPanReversed) {
      digitalWrite(panRight, HIGH);
    }

    count++;
  }

  if (bounce == 7 && stage == 0 && ps2x.ButtonReleased(PSB_L3)) {

    digitalWrite(swingRight, LOW);
    digitalWrite(panLeft, LOW);
    digitalWrite(swingLeft, LOW);
    digitalWrite(panRight, LOW);

    mocoDistance = count;
    count = 0;
    stage = 1;
  }

  if (bounce == 7 && stage == 1) {

    if (count <= mocoDistance) {

      if (!isSwingReversed) {
        digitalWrite(swingRight, LOW);
      }
      if (isSwingReversed) {
        digitalWrite(swingLeft, LOW);
      }
      if (!isPanReversed) {
        digitalWrite(panLeft, LOW);
      }
      if (isPanReversed) {
        digitalWrite(panRight, LOW);
      }

      if (!isSwingReversed) {
        digitalWrite(swingLeft, HIGH);
      }
      if (isSwingReversed) {
        digitalWrite(swingRight, HIGH);
      }
      if (!isPanReversed) {
        digitalWrite(panRight, HIGH);
      }
      if (isPanReversed) {
        digitalWrite(panLeft, HIGH);
      }

      count++;
    }

    if (count >= mocoDistance) {

      if (!isSwingReversed) {
        digitalWrite(swingLeft, LOW);
      }
      if (isSwingReversed) {
        digitalWrite(swingRight, LOW);
      }
      if (!isPanReversed) {
        digitalWrite(panRight, LOW);
      }
      if (isPanReversed) {
        digitalWrite(panLeft, LOW);
      }

      if (!isSwingReversed) {
        digitalWrite(swingRight, HIGH);
      }
      if (isSwingReversed) {
        digitalWrite(swingLeft, HIGH);
      }
      if (!isPanReversed) {
        digitalWrite(panLeft, HIGH);
      }
      if (isPanReversed) {
        digitalWrite(panRight, HIGH);
      }

      count++;
    }
    if (count >= mocoDistance * 2) {
      count = 0;
    }
  }

  // Bounce 8: boom down
  if (bounce == 0 && leftStickYvalue == 255 && ps2x.ButtonReleased(PSB_START)) {
    bounce = 8;
    Serial.println ("bounce 8");
  }

  if (bounce == 8 && stage == 0) {

    if (!isLiftReversed) {
      digitalWrite(liftDown, HIGH);
    }
    if (isLiftReversed) {
      digitalWrite(liftUp, HIGH);
    }

    if (!isTiltReversed) {
      digitalWrite(tiltUp, HIGH);
    }
    if (isTiltReversed) {
      digitalWrite(tiltDown, HIGH);
    }

    count++;

  }

  if (bounce == 8 && stage == 0 && ps2x.ButtonReleased(PSB_L3)) {

    digitalWrite(liftUp, LOW);
    digitalWrite(tiltDown, LOW);
    digitalWrite(liftDown, LOW);
    digitalWrite(tiltUp, LOW);

    mocoDistance = count;
    count = 0;
    stage = 1;

  }

  if (bounce == 8 && stage == 1) {
    if (count <= mocoDistance) {

      if (!isLiftReversed) {
        digitalWrite(liftDown, LOW);
      }
      if (isLiftReversed) {
        digitalWrite(liftUp, LOW);
      }

      if (!isTiltReversed) {
        digitalWrite(tiltUp, LOW);
      }
      if (isTiltReversed) {
        digitalWrite(tiltDown, LOW);
      }

      if (!isLiftReversed) {
        digitalWrite(liftUp, HIGH);
      }
      if (isLiftReversed) {
        digitalWrite(liftDown, HIGH);
      }

      if (!isTiltReversed) {
        digitalWrite(tiltDown, HIGH);
      }
      if (isTiltReversed) {
        digitalWrite(tiltUp, HIGH);
      }
      count++;
    }

    if (count >= mocoDistance) {

      if (!isLiftReversed) {
        digitalWrite(liftUp, LOW);
      }
      if (isLiftReversed) {
        digitalWrite(liftDown, LOW);
      }

      if (!isTiltReversed) {
        digitalWrite(tiltDown, LOW);
      }
      if (isTiltReversed) {
        digitalWrite(tiltUp, LOW);
      }

      if (!isLiftReversed) {
        digitalWrite(liftDown, HIGH);
      }
      if (isLiftReversed) {
        digitalWrite(liftUp, HIGH);
      }

      if (!isTiltReversed) {
        digitalWrite(tiltUp, HIGH);
      }
      if (isTiltReversed) {
        digitalWrite(tiltDown, HIGH);
      }
      count++;
    }

    if (count >= mocoDistance * 2) {
      count = 0;
    }
  }
}

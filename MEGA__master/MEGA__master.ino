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

int swingInMotion = 0;
int liftInMotion = 0;

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
constexpr int PAN_STOP_NONE = 0;
constexpr int PAN_STOP_ACTIVE = 1;
constexpr int PAN_STOP_TRIM = 2;

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
                                    int& soloState, int& motionState) {
  if (soloState == 0 && ps2x.Button(buttonCode)) {
    if (DEBUG_EDGE_EVENTS && (buttonCode == PSB_PAD_LEFT || buttonCode == PSB_PAD_RIGHT)) {
      Serial.print("COMBINED press: ");
      Serial.println(buttonCode == PSB_PAD_LEFT ? "PAD_LEFT" : "PAD_RIGHT");
    }
    motionState = 1;
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
    motionState = 0;
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
  if (swingInMotion == 1) {
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
  if (swingInMotion == 0 && panStop == PAN_STOP_ACTIVE && rightStickXvalue == STICK_CENTER) {
    digitalWrite(panLeft, LOW);
    digitalWrite(panRight, LOW);
    digitalWrite(panSpeedUpOnly, LOW);
    digitalWrite(panSpeedDownOnly, LOW);
    panStop = PAN_STOP_NONE;
  }
}

void handlePanAxis() {
  if (swingInMotion == 0) {
    if (rightStickXvalue == STICK_MIN) {
      activatePanOnlyMotion(panLeft, panRight, "panleftnonly with top speed");
    } else if (rightStickXvalue == STICK_MAX) {
      activatePanOnlyMotion(panRight, panLeft, "panrightonly with top speed");
    }
  }

  stopPanOnlyMotionAtCenter();
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

  ps2x.read_gamepad(false, vibrate); // unnecessary vibration

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
  if (ps2x.Button(PSB_SELECT) && ps2x.Button(PSB_PAD_UP)) {
    if (!isLiftReversed) {
      digitalWrite(liftUp, HIGH);
    }
    if (isLiftReversed) {
      digitalWrite(liftDown, HIGH);
    }
    liftSoloMode = 1;
  }

  if (liftSoloMode == 1 && ps2x.ButtonReleased(PSB_PAD_UP)) {
    digitalWrite(liftUp, LOW);
    digitalWrite(liftDown, LOW);
    liftSoloMode = 0;
  }

  // lift DOWN (no tilt)
  if (ps2x.Button(PSB_SELECT) && ps2x.Button(PSB_PAD_DOWN)) {
    if (!isLiftReversed) {
      digitalWrite(liftDown, HIGH);
    }
    if (isLiftReversed) {
      digitalWrite(liftUp, HIGH);
    }
    liftSoloMode = 1;
  }

  if (liftSoloMode == 1  && ps2x.ButtonReleased(PSB_PAD_DOWN)) {
    digitalWrite(liftDown, LOW);
    digitalWrite(liftUp, LOW);
    liftSoloMode = 0;
  }

  // lift UP + tilt DOWN
  ///////////////////

  if (liftSoloMode == 0 && ps2x.Button(PSB_PAD_UP)) {
    liftInMotion = 1;
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
  }

  if (liftSoloMode == 0 && ps2x.ButtonReleased(PSB_PAD_UP)) {
    digitalWrite(liftUp, LOW);
    digitalWrite(tiltDown, LOW);
    digitalWrite(liftDown, LOW);
    digitalWrite(tiltUp, LOW);
    liftInMotion = 0;
  }

  // lift DOWN + tilt UP
  ///////////////////
  if (liftSoloMode == 0 && ps2x.Button(PSB_PAD_DOWN)) {
    liftInMotion = 1;

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
  }

  if (liftSoloMode == 0 && ps2x.ButtonReleased(PSB_PAD_DOWN)) {
    digitalWrite(liftUp, LOW);
    digitalWrite(tiltDown, LOW);
    digitalWrite(liftDown, LOW);
    digitalWrite(tiltUp, LOW);
    liftInMotion = 0;
  }

  // TODO: Consider consolidating liftInMotion checks.

  if (liftInMotion == 1 && rightStickYvalue == 255) {
    Serial.println("tiltSpeedDownOnly");
    digitalWrite(tiltSpeedDownOnly, HIGH);
    tiltStop = 2;
  }

  if (liftInMotion == 1 && rightStickYvalue == 0) {
    Serial.println("tiltspeedupnonly");
    digitalWrite(tiltSpeedUpOnly, HIGH);
    tiltStop = 2;
  }
  // TODO: Consider whether this should also check liftInMotion.
  if (tiltStop == 2 && rightStickYvalue == 128) {
    digitalWrite(tiltSpeedUpOnly, LOW);
    digitalWrite(tiltSpeedDownOnly, LOW);
    tiltStop = 0;
  }

  /////////////////////////////
  //2nd AXIS (CAMERA tilt)
  ////////////////////////////

  if (liftInMotion == 0  && rightStickYvalue == 255) {
    Serial.println("tiltdownonly");
    //signal to nano to use top speed
    digitalWrite(tiltSpeedUpOnly, HIGH); //signal to nano to use top speed
    digitalWrite(tiltSpeedDownOnly, HIGH); //signal to nano to use top speed

    if (!isTiltReversed) {
      digitalWrite(tiltDown, HIGH);
    }
    if (isTiltReversed) {
      digitalWrite(tiltUp, HIGH);
    }
    tiltStop = 1;
  }

  if (liftInMotion == 0 && rightStickYvalue == 0) {
    Serial.println("tiltuponly");

    digitalWrite(tiltSpeedUpOnly, HIGH); //signal to nano to use top speed
    digitalWrite(tiltSpeedDownOnly, HIGH); //signal to nano to use top speed
    if (!isTiltReversed) {
      digitalWrite(tiltUp, HIGH);
    }
    if (isTiltReversed) {
      digitalWrite(tiltDown, HIGH);
    }
    tiltStop = 1;
  }

  if (liftInMotion == 0 && tiltStop == 1 && rightStickYvalue == 128) {
    digitalWrite(tiltUp, LOW);
    digitalWrite(tiltDown, LOW);
    digitalWrite(tiltSpeedUpOnly, LOW);
    digitalWrite(tiltSpeedDownOnly, LOW);
    tiltStop = 0;
  }

  /////////////////////////////
  //5th AXIS (Camera focus)
  ////////////////////////////

  if (ps2x.Button(PSB_TRIANGLE)) {
    if (!isFocusReversed) {
      digitalWrite(focusLeft, HIGH);
    }
    if (isFocusReversed) {
      digitalWrite(focusRight, HIGH);
    }

  }
  if (ps2x.ButtonReleased(PSB_TRIANGLE)) {
    digitalWrite(focusLeft, LOW);
    digitalWrite(focusRight, LOW);
  }

  if (ps2x.Button(PSB_CROSS)) {
    if (!isFocusReversed) {
      digitalWrite(focusRight, HIGH);
    }
    if (isFocusReversed) {
      digitalWrite(focusLeft, HIGH);
    }
  }

  if (ps2x.ButtonReleased(PSB_CROSS)) {
    digitalWrite(focusRight, LOW);
    digitalWrite(focusLeft, LOW);
  }

  if (ps2x.Button(PSB_SQUARE)) {
    digitalWrite(focusSpeedDown, HIGH);
  }
  if (ps2x.ButtonReleased(PSB_SQUARE)) {
    digitalWrite(focusSpeedDown, LOW);
  }

  if (ps2x.Button(PSB_CIRCLE)) {
    digitalWrite(focusSpeedUp, HIGH);
  }
  if (ps2x.ButtonReleased(PSB_CIRCLE)) {
    digitalWrite(focusSpeedUp, LOW);
  }

  ////////////////////////////////////////////////////
  /////////////////////Timelapse/////////////////////
  //////////////////////////////////////////////////

  ///////swing left boom down
  ///////////////////////////////////

  if (timelapseMode == 0 && leftStickXvalue < 123 && leftStickYvalue > 133 && ps2x.ButtonReleased(PSB_SELECT)) {
    timelapseMode = 1; //swing left boom down
  }

  if (timelapseMode == 1) {
    Serial.println("timelapse mode 1");
    digitalWrite(trigger, LOW);
    delay(interval / 2);
    digitalWrite(trigger, HIGH);
    delay(interval / 2);

    Serial.println("turning on timelapse 1 now");
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

    delay(stepDist);

    Serial.println("turning off timelapse 1 now");
    digitalWrite(swingLeft, LOW);
    digitalWrite(swingRight, LOW);
    digitalWrite(panRight, LOW);
    digitalWrite(panLeft, LOW);
    digitalWrite(liftDown, LOW);
    digitalWrite(liftUp, LOW);
    digitalWrite(tiltUp, LOW);
    digitalWrite(tiltDown, LOW);
  }

  ///////swing LEFT and lift UP
  ///////////////////////////////////
  if (timelapseMode == 0 && leftStickXvalue < 123 && leftStickYvalue < 123 && ps2x.ButtonReleased(PSB_SELECT)) {
    timelapseMode = 2; //swing left boom up
  }

  if (timelapseMode == 2) {
    Serial.println("timelapse mode 2");
    digitalWrite(trigger, LOW);
    delay(interval / 2);
    digitalWrite(trigger, HIGH);
    delay(interval / 2);

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

    delay(stepDist);
    digitalWrite(swingLeft, LOW);
    digitalWrite(swingRight, LOW);
    digitalWrite(panRight, LOW);
    digitalWrite(panLeft, LOW);
    digitalWrite(liftDown, LOW);
    digitalWrite(liftUp, LOW);
    digitalWrite(tiltUp, LOW);
    digitalWrite(tiltDown, LOW);
  }

  ///////swing right and boomup
  ///////////////////////////////////
  if (timelapseMode == 0 && leftStickXvalue > 133 && leftStickYvalue < 123 && ps2x.ButtonReleased(PSB_SELECT)) {
    timelapseMode = 3;  //swing right and boom up
  }

  if (timelapseMode == 3) {
    Serial.println("timelapse mode 3");
    digitalWrite(trigger, LOW);
    delay(interval / 2);
    digitalWrite(trigger, HIGH);
    delay(interval / 2);

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

    delay(stepDist);

    digitalWrite(swingLeft, LOW);
    digitalWrite(swingRight, LOW);
    digitalWrite(panRight, LOW);
    digitalWrite(panLeft, LOW);
    digitalWrite(liftDown, LOW);
    digitalWrite(liftUp, LOW);
    digitalWrite(tiltUp, LOW);
    digitalWrite(tiltDown, LOW);
  }

  ///////swing right and boomdown
  ///////////////////////////////////
  if (timelapseMode == 0 && leftStickXvalue > 133 && leftStickYvalue > 133 && ps2x.ButtonReleased(PSB_SELECT)) {
    timelapseMode = 4; //swing right and boomdown
  }

  if (timelapseMode == 4) {
    Serial.println("timelapse mode 4");
    digitalWrite(trigger, LOW);
    delay(interval / 2);
    digitalWrite(trigger, HIGH);
    delay(interval / 2);

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

    delay(stepDist);
    digitalWrite(swingLeft, LOW);
    digitalWrite(swingRight, LOW);
    digitalWrite(panRight, LOW);
    digitalWrite(panLeft, LOW);
    digitalWrite(liftDown, LOW);
    digitalWrite(liftUp, LOW);
    digitalWrite(tiltUp, LOW);
    digitalWrite(tiltDown, LOW);
  }

  ///////swing left
  ///////////////////////////////////

  if (timelapseMode == 0 && leftStickXvalue == 0 && ps2x.ButtonReleased(PSB_SELECT)) {
    timelapseMode = 5; //swing left
  }

  if (timelapseMode == 5) {
    Serial.println("timelapse mode 5");
    digitalWrite(trigger, LOW);
    delay(interval / 2);
    digitalWrite(trigger, HIGH);
    delay(interval / 2);

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

    delay(stepDist);
    digitalWrite(swingLeft, LOW);
    digitalWrite(swingRight, LOW);
    digitalWrite(panRight, LOW);
    digitalWrite(panLeft, LOW);

  }

  ///////lift up
  ///////////////////////////////////
  if (timelapseMode == 0 && leftStickYvalue == 0 && ps2x.ButtonReleased(PSB_SELECT)) {
    timelapseMode = 6; // boom up
  }

  if (timelapseMode == 6) {
    Serial.println("timelapse mode 6");
    digitalWrite(trigger, LOW);
    delay(interval / 2);
    digitalWrite(trigger, HIGH);
    delay(interval / 2);

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

    delay(stepDist);
    digitalWrite(liftDown, LOW);
    digitalWrite(liftUp, LOW);
    digitalWrite(tiltUp, LOW);
    digitalWrite(tiltDown, LOW);
  }

  ///////swing right
  ///////////////////////////////////
  if (timelapseMode == 0 && leftStickXvalue == 255 && ps2x.ButtonReleased(PSB_SELECT)) {
    timelapseMode = 7;  //swing right
  }

  if (timelapseMode == 7) {
    Serial.println("timelapse mode 7");
    digitalWrite(trigger, LOW);
    delay(interval / 2);
    digitalWrite(trigger, HIGH);
    delay(interval / 2);

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

    delay(stepDist);
    digitalWrite(swingLeft, LOW);
    digitalWrite(swingRight, LOW);
    digitalWrite(panRight, LOW);
    digitalWrite(panLeft, LOW);

  }

  /////lift DOWN
  ///////////////
  if (timelapseMode == 0 && leftStickYvalue == 255 && ps2x.ButtonReleased(PSB_SELECT)) {
    timelapseMode = 8;
  }

  if (timelapseMode == 8) {
    Serial.println("timelapse mode 8");
    digitalWrite(trigger, LOW);
    delay(interval / 2);
    digitalWrite(trigger, HIGH);
    delay(interval / 2);

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

    delay(stepDist);
    digitalWrite(liftDown, LOW);
    digitalWrite(liftUp, LOW);
    digitalWrite(tiltUp, LOW);
    digitalWrite(tiltDown, LOW);
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////MOCO moves
  ///////////////////////////////////////////////////////////////////////////////////////////////////////

//turn off bounce?

  /////Bounce 1
  ////////swing LEFT boom DOWN
  ///////////////////////////

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

  /////Bounce2
  ///////swing LEFT Boom UP
  /////////////////////////

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

  /////Bounce3
  ///////swing RIGHT Boom UP
  /////////////////////////
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

  //Bounce 4
  /////swing RIGHT and boom DOWN
  /////////////////////////////
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

  //Bounce 5
  //////////swing LEFT
  ////////////////////////////////////

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

  //Bounce 6
  /////lift UP
  /////////////////
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

  ///Bounce 7
  ///////////swing RIGHT
  ///////////////////////

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

  /////Bounce 8
  /////////////Lift DOWN
  //////////////////////
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

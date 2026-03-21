#include <PS2X_lib.h>  //for v1.6

#define PS2_DAT 10
#define PS2_CMD 9
#define PS2_SEL 8
#define PS2_CLK 11

#define pressures false
#define rumble false

PS2X ps2x;
int error = 0;
byte type = 0;
byte vibrate = 0;

// Button Settings
int swingLeft = 24;
int swingRight = 25;
int swingSpeedUp = 26;
int swingSpeedDown = 27;
int swingSoloMode = 0;

int panLeft = 46;
int panRight = 48;
int panSpeedUp = 50;
int panSpeedDown = 52;
int panStop = 0;
int speedStop = 0;

int panSpeedUpOnly = 29; //lower
int panSpeedDownOnly = 45; //higher

int liftDown = 30;
int liftUp = 31;
int liftSpeedUp = 32;
int liftSpeedDown = 33;
int liftSoloMode = 0;

int tiltSpeedUpOnly = 42; //lower val
int tiltSpeedDownOnly = 44; //higher val

int tiltDown = 34;
int tiltUp = 36;
int tiltSpeedUp = 38;
int tiltSpeedDown = 40;
int tiltStop = 0;

int focusLeft = 47;
int focusRight = 49;
int focusSpeedUp = 51;
int focusSpeedDown = 53;

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
int intervalDelay;
int stepDist = 100;
int trigger = 28;

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

bool isSwingReversed = false;
bool isPanReversed = false;
bool isLiftReversed = false;
bool isTiltReversed = false;
bool isFocusReversed = false;
int delayTime;

void setup() {

  //Serial.begin(57600);

  const int outputPins[] = {
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

  interval = intervalSeconds * 1000;



  delay(300);

  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);

  if (error == 0) {
    Serial.print("Found Controller, configured successful ");
    Serial.print("pressures = ");
    if (pressures)
      Serial.println("true ");
    else
      Serial.println("false");
    Serial.print("rumble = ");
    if (rumble)
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

  type = ps2x.readType();
  switch (type) {
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





void loop() {

  if (error == 1) //skip loop if no controller found
    return;

  if (type == 2) { //Guitar Hero Controller
  }
  else { //DualShock Controller
    ps2x.read_gamepad(false, vibrate); //uneccessary vibration

    isSwingReversed = (digitalRead(DIP_SWITCH_1) == HIGH);
    isPanReversed = (digitalRead(DIP_SWITCH_2) == HIGH);
    isLiftReversed = (digitalRead(DIP_SWITCH_3) == HIGH);
    isTiltReversed = (digitalRead(DIP_SWITCH_4) == HIGH);
    isFocusReversed = (digitalRead(DIP_SWITCH_5) == HIGH);

    rightStickYvalue = ps2x.Analog(PSS_RY);
    rightStickXvalue = ps2x.Analog(PSS_RX);
    leftStickYvalue  = ps2x.Analog(PSS_LY);
    leftStickXvalue  = ps2x.Analog(PSS_LX);


    //////////////R1 and R2 Buttons
    if (ps2x.Button(PSB_R1)) {
      digitalWrite(liftSpeedUp, HIGH);
      digitalWrite(tiltSpeedUp, HIGH);
    }
    if (ps2x.ButtonReleased(PSB_R1)) {
      digitalWrite(liftSpeedUp, LOW);
      digitalWrite(tiltSpeedUp, LOW);
    }

    if (ps2x.Button(PSB_R2)) {
      digitalWrite(liftSpeedDown, HIGH);
      digitalWrite(tiltSpeedDown, HIGH);
    }

    if (ps2x.ButtonReleased(PSB_R2)) {
      digitalWrite(liftSpeedDown, LOW);
      digitalWrite(tiltSpeedDown, LOW);
    }

    ///////////L1 ad L2 Buttons
    if (ps2x.Button(PSB_L1)) {
      digitalWrite(panSpeedUp, HIGH);
      digitalWrite(swingSpeedUp, HIGH);
    }
    if (ps2x.ButtonReleased(PSB_L1)) {
      digitalWrite(panSpeedUp, LOW);
      digitalWrite(swingSpeedUp, LOW);
    }

    if (ps2x.Button(PSB_L2)) {
      digitalWrite(panSpeedDown, HIGH);
      digitalWrite(swingSpeedDown, HIGH);
    }
    if (ps2x.ButtonReleased(PSB_L2)) {
      digitalWrite(panSpeedDown, LOW);
      digitalWrite(swingSpeedDown, LOW);
    }

    /////////////////////////////
    //1st AXIS (BOOM SWING)
    ////////////////////////////

    ///////////swingLeft (no pan)
    /////////////////////////////////
    if (ps2x.Button(PSB_SELECT) && ps2x.Button(PSB_PAD_LEFT)) {
      if (!isSwingReversed) {
        digitalWrite(swingLeft, HIGH);
      }
      if (isSwingReversed) {
        digitalWrite(swingRight, HIGH);
      }
      swingSoloMode = 1;
    }

    if (swingSoloMode == 1 && ps2x.ButtonReleased(PSB_PAD_LEFT)) {
      digitalWrite(swingLeft, LOW);
      digitalWrite(swingRight, LOW);
      swingSoloMode = 0;
    }

    ///////////swingRight (no pan)
    /////////////////////////////////
    if (ps2x.Button(PSB_SELECT) && ps2x.Button(PSB_PAD_RIGHT)) {
      if (!isSwingReversed) {
        digitalWrite(swingRight, HIGH);
      }
      if (isSwingReversed) {
        digitalWrite(swingLeft, HIGH);
      }
      swingSoloMode = 1;

    }
    if (swingSoloMode == 1 && ps2x.ButtonReleased(PSB_PAD_RIGHT)) {
      digitalWrite(swingRight, LOW);
      digitalWrite(swingLeft, LOW);
      swingSoloMode = 0;
    }

    ///////////swingLeft panRight
    /////////////////////////////////
    if (swingSoloMode == 0 && ps2x.Button(PSB_PAD_LEFT)) {
      swingInMotion = 1;
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
    }

    if (swingSoloMode == 0 && ps2x.ButtonReleased(PSB_PAD_LEFT)) {
      digitalWrite(swingLeft, LOW);
      digitalWrite (panRight, LOW);
      digitalWrite(swingRight, LOW);
      digitalWrite (panLeft, LOW);
      swingInMotion = 0;
    }


    ///////////swingRight panLeft
    //////////////////////////////
    if (swingSoloMode == 0 && ps2x.Button(PSB_PAD_RIGHT)) {
      swingInMotion = 1;
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
    }

    if (swingSoloMode == 0 && ps2x.ButtonReleased(PSB_PAD_RIGHT)) {
      digitalWrite(swingRight, LOW);
      digitalWrite (panLeft, LOW);
      digitalWrite(swingLeft, LOW);
      digitalWrite (panRight, LOW);
      swingInMotion = 0;
    }

    //should all these below be within 1 if statement for swingmotion?
    if (swingInMotion == 1 && rightStickXvalue == 0) {
      digitalWrite(panSpeedDownOnly, HIGH);
      Serial.println("panSpeedDownOnly");
      panStop = 2;
    }

    if (swingInMotion == 1 && rightStickXvalue == 255) {
      digitalWrite(panSpeedUpOnly, HIGH);
      Serial.println("panSpeedUpOnly");
      panStop = 2;
    }

    if (panStop == 2 && rightStickXvalue == 128  ) {
      digitalWrite(panSpeedUpOnly, LOW);
      digitalWrite(panSpeedDownOnly, LOW);
      panStop = 0;
    }

    /////////////////////////////
    //2nd AXIS (CAMERA PAN)
    ////////////////////////////

    if (swingInMotion == 0 &&  rightStickXvalue == 0) {
      Serial.println("panleftnonly with top speed");

      digitalWrite(panSpeedUpOnly, HIGH); //signal to nano to use top speed
      digitalWrite(panSpeedDownOnly, HIGH);//signal to nano to use top speed

      if (!isPanReversed) {
        digitalWrite(panLeft, HIGH);
      }
      if (isPanReversed) {
        digitalWrite(panRight, HIGH);
      }
      panStop = 1;
    }

    if (swingInMotion == 0 && rightStickXvalue == 255) {
      Serial.println("panrightonly with top speed");
      digitalWrite(panSpeedUpOnly, HIGH); //signal to nano to use top speed
      digitalWrite(panSpeedDownOnly, HIGH);//signal to nano to use top speed

      if (!isPanReversed) {
        digitalWrite(panRight, HIGH);
      }
      if (isPanReversed) {
        digitalWrite(panLeft, HIGH);
      }
      panStop = 1;
    }

    if (swingInMotion == 0 && panStop == 1 && rightStickXvalue == 128) {
      digitalWrite(panLeft, LOW);
      digitalWrite(panRight, LOW);
      digitalWrite(panSpeedUpOnly, LOW);
      digitalWrite(panSpeedDownOnly, LOW);
      panStop = 0;
    }


    /////////////////////////////
    //3rd AXIS (BOOM LIFT)
    ////////////////////////////

    //lift UP (no tilt)
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

    //lift DOWN (no tilt)
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


    //lift UP tilt DOWN
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

    //lift DOWN tilt UP
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

    //should this all be inn 1 liftInMotion if statement?

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
    //should below also be a liftInMotion?
    if (tiltStop == 2 && rightStickYvalue == 128  ) {
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

    if  (timelapseMode == 0 && leftStickXvalue < 123 && leftStickYvalue > 133 && ps2x.ButtonReleased(PSB_SELECT)) {
      timelapseMode = 1; //swing left boom down
    }

    if (timelapseMode == 1) {
      Serial.println("timelapse mode 1");
      digitalWrite(trigger, LOW);
      delay (interval / 2);
      digitalWrite(trigger, HIGH);
      delay (interval / 2);

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
      delay (interval / 2);
      digitalWrite(trigger, HIGH);
      delay (interval / 2);

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
      delay (interval / 2);
      digitalWrite(trigger, HIGH);
      delay (interval / 2);


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
      delay (interval / 2);
      digitalWrite(trigger, HIGH);
      delay (interval / 2);



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

    if  (timelapseMode == 0 && leftStickXvalue == 0 && ps2x.ButtonReleased(PSB_SELECT)) {
      timelapseMode = 5; //swing left
    }

    if (timelapseMode == 5) {
      Serial.println("timelapse mode 5");
      digitalWrite(trigger, LOW);
      delay (interval / 2);
      digitalWrite(trigger, HIGH);
      delay (interval / 2);

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
      delay (interval / 2);
      digitalWrite(trigger, HIGH);
      delay (interval / 2);

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
      delay (interval / 2);
      digitalWrite(trigger, HIGH);
      delay (interval / 2);

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
      delay (interval / 2);
      digitalWrite(trigger, HIGH);
      delay (interval / 2);


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
  delay(10);
}

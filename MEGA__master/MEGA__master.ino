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

//timelapse variables
///////////////////
int timelapseMode = 0;
int intervalSeconds = 15;
int interval;
int intervalDelay;
int stepDist = 100;
int trigger = 28;

//motion control variables
int bounce = 0;
int count = 0;
int mocoDistance = 0;
int stage = 0;

int dipSwitch1 = 35;
int dipSwitch2 = 43;
int dipSwitch3 = 37;
int dipSwitch4 = 39;
int dipSwitch5 = 41;

int swingSwitch = 0;
int panSwitch = 0;
int liftSwitch = 0;
int tiltSwitch = 0;
int focusSwitch = 0;
int delayTime;

void setup() {

  //Serial.begin(57600);

  pinMode (swingLeft, OUTPUT);
  pinMode (swingRight, OUTPUT);
  pinMode (swingSpeedUp, OUTPUT);
  pinMode (swingSpeedDown, OUTPUT);
  pinMode (liftDown, OUTPUT);
  pinMode (liftUp, OUTPUT);
  pinMode (liftSpeedUp, OUTPUT);
  pinMode (liftSpeedDown, OUTPUT);
  pinMode (panLeft, OUTPUT);
  pinMode (panRight, OUTPUT);
  pinMode (panSpeedUp, OUTPUT);
  pinMode (panSpeedDown, OUTPUT);
  pinMode (tiltDown, OUTPUT);
  pinMode (tiltUp, OUTPUT);
  pinMode (tiltSpeedUp, OUTPUT);
  pinMode (tiltSpeedDown, OUTPUT);
  pinMode (focusLeft, OUTPUT);
  pinMode (focusRight, OUTPUT);
  pinMode (focusSpeedUp, OUTPUT);
  pinMode (focusSpeedDown, OUTPUT);

  pinMode(trigger, OUTPUT);
  interval = intervalSeconds * 1000;

  pinMode (dipSwitch1, INPUT_PULLUP);
  pinMode (dipSwitch2, INPUT_PULLUP);
  pinMode (dipSwitch3, INPUT_PULLUP);
  pinMode (dipSwitch4, INPUT_PULLUP);
  pinMode (dipSwitch5, INPUT_PULLUP);

  pinMode (tiltSpeedUpOnly, OUTPUT);
  pinMode (tiltSpeedDownOnly, OUTPUT);

  pinMode (panSpeedUpOnly, OUTPUT);
  pinMode (panSpeedDownOnly, OUTPUT);



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

    swingSwitch = digitalRead(dipSwitch1);
    panSwitch = digitalRead(dipSwitch2);
    liftSwitch = digitalRead(dipSwitch3);
    tiltSwitch = digitalRead(dipSwitch4);
    focusSwitch =  digitalRead(dipSwitch5);

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
      if (swingSwitch == 0) {
        digitalWrite(swingLeft, HIGH);
      }
      if (swingSwitch == 1) {
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
      if (swingSwitch == 0) {
        digitalWrite(swingRight, HIGH);
      }
      if (swingSwitch == 1) {
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
      if (swingSwitch == 0) {
        digitalWrite(swingLeft, HIGH);
      }
      if (swingSwitch == 1) {
        digitalWrite(swingRight, HIGH);
      }
      if (panSwitch == 0) {
        digitalWrite(panRight, HIGH);
      }
      if (panSwitch == 1) {
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
      if (swingSwitch == 0) {
        digitalWrite(swingRight, HIGH);
      }
      if (swingSwitch == 1) {
        digitalWrite(swingLeft, HIGH);
      }
      if (panSwitch == 0) {
        digitalWrite(panLeft, HIGH);
      }
      if (panSwitch == 1) {
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

      if (panSwitch == 0) {
        digitalWrite(panLeft, HIGH);
      }
      if (panSwitch == 1) {
        digitalWrite(panRight, HIGH);
      }
      panStop = 1;
    }

    if (swingInMotion == 0 && rightStickXvalue == 255) {
      Serial.println("panrightonly with top speed");
      digitalWrite(panSpeedUpOnly, HIGH); //signal to nano to use top speed
      digitalWrite(panSpeedDownOnly, HIGH);//signal to nano to use top speed

      if (panSwitch == 0) {
        digitalWrite(panRight, HIGH);
      }
      if (panSwitch == 1) {
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
      if (liftSwitch == 0) {
        digitalWrite(liftUp, HIGH);
      }
      if (liftSwitch == 1) {
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
      if (liftSwitch == 0) {
        digitalWrite(liftDown, HIGH);
      }
      if (liftSwitch == 1) {
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
      if (liftSwitch == 0) {
        digitalWrite(liftUp, HIGH);
      }
      if (liftSwitch == 1) {
        digitalWrite(liftDown, HIGH);
      }

      if (tiltSwitch == 0) {
        digitalWrite(tiltDown, HIGH);
      }
      if (tiltSwitch == 1) {
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

      if (liftSwitch == 0) {
        digitalWrite(liftDown, HIGH);
      }
      if (liftSwitch == 1) {
        digitalWrite(liftUp, HIGH);
      }

      if (tiltSwitch == 0) {
        digitalWrite(tiltUp, HIGH);
      }
      if (tiltSwitch == 1) {
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

      if (tiltSwitch == 0) {
        digitalWrite(tiltDown, HIGH);
      }
      if (tiltSwitch == 1) {
        digitalWrite(tiltUp, HIGH);
      }
      tiltStop = 1;
    }

    if (liftInMotion == 0 && rightStickYvalue == 0) {
      Serial.println("tiltuponly");

      digitalWrite(tiltSpeedUpOnly, HIGH); //signal to nano to use top speed
      digitalWrite(tiltSpeedDownOnly, HIGH); //signal to nano to use top speed
      if (tiltSwitch == 0) {
        digitalWrite(tiltUp, HIGH);
      }
      if (tiltSwitch == 1) {
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
      if (focusSwitch == 0) {
        digitalWrite(focusLeft, HIGH);
      }
      if (focusSwitch == 1) {
        digitalWrite(focusRight, HIGH);
      }

    }
    if (ps2x.ButtonReleased(PSB_TRIANGLE)) {
      digitalWrite(focusLeft, LOW);
      digitalWrite(focusRight, LOW);
    }

    if (ps2x.Button(PSB_CROSS)) {
      if (focusSwitch == 0) {
        digitalWrite(focusRight, HIGH);
      }
      if (focusSwitch == 1) {
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
      if (swingSwitch == 0) {
        digitalWrite(swingLeft, HIGH);
      }
      if (swingSwitch == 1) {
        digitalWrite(swingRight, HIGH);
      }
      if (panSwitch == 0) {
        digitalWrite(panRight, HIGH);
      }
      if (panSwitch == 1) {
        digitalWrite(panLeft, HIGH);
      }

      if (liftSwitch == 0) {
        digitalWrite(liftDown, HIGH);
      }
      if (liftSwitch == 1) {
        digitalWrite(liftUp, HIGH);
      }

      if (tiltSwitch == 0) {
        digitalWrite(tiltUp, HIGH);
      }
      if (tiltSwitch == 1) {
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

      if (swingSwitch == 0) {
        digitalWrite(swingLeft, HIGH);
      }
      if (swingSwitch == 1) {
        digitalWrite(swingRight, HIGH);
      }
      if (panSwitch == 0) {
        digitalWrite(panRight, HIGH);
      }
      if (panSwitch == 1) {
        digitalWrite(panLeft, HIGH);
      }

      if (liftSwitch == 0) {
        digitalWrite(liftUp, HIGH);
      }
      if (liftSwitch == 1) {
        digitalWrite(liftDown, HIGH);
      }

      if (tiltSwitch == 0) {
        digitalWrite(tiltDown, HIGH);
      }
      if (tiltSwitch == 1) {
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


      if (swingSwitch == 0) {
        digitalWrite(swingRight, HIGH);
      }
      if (swingSwitch == 1) {
        digitalWrite(swingLeft, HIGH);
      }
      if (panSwitch == 0) {
        digitalWrite(panLeft, HIGH);
      }
      if (panSwitch == 1) {
        digitalWrite(panRight, HIGH);
      }

      if (liftSwitch == 0) {
        digitalWrite(liftUp, HIGH);
      }
      if (liftSwitch == 1) {
        digitalWrite(liftDown, HIGH);
      }

      if (tiltSwitch == 0) {
        digitalWrite(tiltDown, HIGH);
      }
      if (tiltSwitch == 1) {
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



      if (swingSwitch == 0) {
        digitalWrite(swingRight, HIGH);
      }
      if (swingSwitch == 1) {
        digitalWrite(swingLeft, HIGH);
      }
      if (panSwitch == 0) {
        digitalWrite(panLeft, HIGH);
      }
      if (panSwitch == 1) {
        digitalWrite(panRight, HIGH);
      }

      if (liftSwitch == 0) {
        digitalWrite(liftDown, HIGH);
      }
      if (liftSwitch == 1) {
        digitalWrite(liftUp, HIGH);
      }

      if (tiltSwitch == 0) {
        digitalWrite(tiltUp, HIGH);
      }
      if (tiltSwitch == 1) {
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

      if (swingSwitch == 0) {
        digitalWrite(swingLeft, HIGH);
      }
      if (swingSwitch == 1) {
        digitalWrite(swingRight, HIGH);
      }
      if (panSwitch == 0) {
        digitalWrite(panRight, HIGH);
      }
      if (panSwitch == 1) {
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

      if (liftSwitch == 0) {
        digitalWrite(liftUp, HIGH);
      }
      if (liftSwitch == 1) {
        digitalWrite(liftDown, HIGH);
      }

      if (tiltSwitch == 0) {
        digitalWrite(tiltDown, HIGH);
      }
      if (tiltSwitch == 1) {
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

      if (swingSwitch == 0) {
        digitalWrite(swingRight, HIGH);
      }
      if (swingSwitch == 1) {
        digitalWrite(swingLeft, HIGH);
      }
      if (panSwitch == 0) {
        digitalWrite(panLeft, HIGH);
      }
      if (panSwitch == 1) {
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


      if (liftSwitch == 0) {
        digitalWrite(liftDown, HIGH);
      }
      if (liftSwitch == 1) {
        digitalWrite(liftUp, HIGH);
      }

      if (tiltSwitch == 0) {
        digitalWrite(tiltUp, HIGH);
      }
      if (tiltSwitch == 1) {
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
      if (swingSwitch == 0) {
        digitalWrite(swingLeft, HIGH);
      }
      if (swingSwitch == 1) {
        digitalWrite(swingRight, HIGH);
      }
      if (panSwitch == 0) {
        digitalWrite(panRight, HIGH);
      }
      if (panSwitch == 1) {
        digitalWrite(panLeft, HIGH);
      }

      if (liftSwitch == 0) {
        digitalWrite(liftDown, HIGH);
      }
      if (liftSwitch == 1) {
        digitalWrite(liftUp, HIGH);
      }

      if (tiltSwitch == 0) {
        digitalWrite(tiltUp, HIGH);
      }
      if (tiltSwitch == 1) {
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
        if (swingSwitch == 0) {
          digitalWrite(swingLeft, LOW);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingRight, LOW);
        }
        if (panSwitch == 0) {
          digitalWrite(panRight, LOW);
        }
        if (panSwitch == 1) {
          digitalWrite(panLeft, LOW);
        }

        if (liftSwitch == 0) {
          digitalWrite(liftDown, LOW);
        }
        if (liftSwitch == 1) {
          digitalWrite(liftUp, LOW);
        }

        if (tiltSwitch == 0) {
          digitalWrite(tiltUp, LOW);
        }
        if (tiltSwitch == 1) {
          digitalWrite(tiltDown, LOW);
        }


        //turn on motors

        if (swingSwitch == 0) {
          digitalWrite(swingRight, HIGH);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingLeft, HIGH);
        }
        if (panSwitch == 0) {
          digitalWrite(panLeft, HIGH);
        }
        if (panSwitch == 1) {
          digitalWrite(panRight, HIGH);
        }

        if (liftSwitch == 0) {
          digitalWrite(liftUp, HIGH);
        }
        if (liftSwitch == 1) {
          digitalWrite(liftDown, HIGH);
        }

        if (tiltSwitch == 0) {
          digitalWrite(tiltDown, HIGH);
        }
        if (tiltSwitch == 1) {
          digitalWrite(tiltUp, HIGH);
        }
        count++;
      }

      if (count >= mocoDistance) {//starting point

        if (swingSwitch == 0) {
          digitalWrite(swingRight, LOW);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingLeft, LOW);
        }
        if (panSwitch == 0) {
          digitalWrite(panLeft, LOW);
        }
        if (panSwitch == 1) {
          digitalWrite(panRight, LOW);
        }

        if (liftSwitch == 0) {
          digitalWrite(liftUp, LOW);
        }
        if (liftSwitch == 1) {
          digitalWrite(liftDown, LOW);
        }

        if (tiltSwitch == 0) {
          digitalWrite(tiltDown, LOW);
        }
        if (tiltSwitch == 1) {
          digitalWrite(tiltUp, LOW);
        }

        //turn motors on

        if (swingSwitch == 0) {
          digitalWrite(swingLeft, HIGH);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingRight, HIGH);
        }
        if (panSwitch == 0) {
          digitalWrite(panRight, HIGH);
        }
        if (panSwitch == 1) {
          digitalWrite(panLeft, HIGH);
        }

        if (liftSwitch == 0) {
          digitalWrite(liftDown, HIGH);
        }
        if (liftSwitch == 1) {
          digitalWrite(liftUp, HIGH);
        }

        if (tiltSwitch == 0) {
          digitalWrite(tiltUp, HIGH);
        }
        if (tiltSwitch == 1) {
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
      if (swingSwitch == 0) {
        digitalWrite(swingLeft, HIGH);
      }
      if (swingSwitch == 1) {
        digitalWrite(swingRight, HIGH);
      }
      if (panSwitch == 0) {
        digitalWrite(panRight, HIGH);
      }
      if (panSwitch == 1) {
        digitalWrite(panLeft, HIGH);
      }

      if (liftSwitch == 0) {
        digitalWrite(liftUp, HIGH);
      }
      if (liftSwitch == 1) {
        digitalWrite(liftDown, HIGH);
      }

      if (tiltSwitch == 0) {
        digitalWrite(tiltDown, HIGH);
      }
      if (tiltSwitch == 1) {
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

        if (swingSwitch == 0) {
          digitalWrite(swingLeft, LOW);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingRight, LOW);
        }
        if (panSwitch == 0) {
          digitalWrite(panRight, LOW);
        }
        if (panSwitch == 1) {
          digitalWrite(panLeft, LOW);
        }

        if (liftSwitch == 0) {
          digitalWrite(liftUp, LOW);
        }
        if (liftSwitch == 1) {
          digitalWrite(liftDown, LOW);
        }

        if (tiltSwitch == 0) {
          digitalWrite(tiltDown, LOW);
        }
        if (tiltSwitch == 1) {
          digitalWrite(tiltUp, LOW);
        }


        //turn motors on

        if (swingSwitch == 0) {
          digitalWrite(swingRight, HIGH);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingLeft, HIGH);
        }
        if (panSwitch == 0) {
          digitalWrite(panLeft, HIGH);
        }
        if (panSwitch == 1) {
          digitalWrite(panRight, HIGH);
        }

        if (liftSwitch == 0) {
          digitalWrite(liftDown, HIGH);
        }
        if (liftSwitch == 1) {
          digitalWrite(liftUp, HIGH);
        }

        if (tiltSwitch == 0) {
          digitalWrite(tiltUp, HIGH);
        }
        if (tiltSwitch == 1) {
          digitalWrite(tiltDown, HIGH);
        }
        count++;
      }

      if (count >= mocoDistance) {

        //turn motors off
        if (swingSwitch == 0) {
          digitalWrite(swingRight, LOW);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingLeft, LOW);
        }
        if (panSwitch == 0) {
          digitalWrite(panLeft, LOW);
        }
        if (panSwitch == 1) {
          digitalWrite(panRight, LOW);
        }

        if (liftSwitch == 0) {
          digitalWrite(liftDown, LOW);
        }
        if (liftSwitch == 1) {
          digitalWrite(liftUp, LOW);
        }

        if (tiltSwitch == 0) {
          digitalWrite(tiltUp, LOW);
        }
        if (tiltSwitch == 1) {
          digitalWrite(tiltDown, LOW);
        }

        //turn motors on
        if (swingSwitch == 0) {
          digitalWrite(swingLeft, HIGH);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingRight, HIGH);
        }
        if (panSwitch == 0) {
          digitalWrite(panRight, HIGH);
        }
        if (panSwitch == 1) {
          digitalWrite(panLeft, HIGH);
        }

        if (liftSwitch == 0) {
          digitalWrite(liftUp, HIGH);
        }
        if (liftSwitch == 1) {
          digitalWrite(liftDown, HIGH);
        }

        if (tiltSwitch == 0) {
          digitalWrite(tiltDown, HIGH);
        }
        if (tiltSwitch == 1) {
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

      if (swingSwitch == 0) {
        digitalWrite(swingRight, HIGH);
      }
      if (swingSwitch == 1) {
        digitalWrite(swingLeft, HIGH);
      }
      if (panSwitch == 0) {
        digitalWrite(panLeft, HIGH);
      }
      if (panSwitch == 1) {
        digitalWrite(panRight, HIGH);
      }

      if (liftSwitch == 0) {
        digitalWrite(liftUp, HIGH);
      }
      if (liftSwitch == 1) {
        digitalWrite(liftDown, HIGH);
      }

      if (tiltSwitch == 0) {
        digitalWrite(tiltDown, HIGH);
      }
      if (tiltSwitch == 1) {
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
        if (swingSwitch == 0) {
          digitalWrite(swingRight, LOW);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingLeft, LOW);
        }
        if (panSwitch == 0) {
          digitalWrite(panLeft, LOW);
        }
        if (panSwitch == 1) {
          digitalWrite(panRight, LOW);
        }

        if (liftSwitch == 0) {
          digitalWrite(liftUp, LOW);
        }
        if (liftSwitch == 1) {
          digitalWrite(liftDown, LOW);
        }

        if (tiltSwitch == 0) {
          digitalWrite(tiltDown, LOW);
        }
        if (tiltSwitch == 1) {
          digitalWrite(tiltUp, LOW);
        }


        //motor on
        if (swingSwitch == 0) {
          digitalWrite(swingLeft, HIGH);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingRight, HIGH);
        }
        if (panSwitch == 0) {
          digitalWrite(panRight, HIGH);
        }
        if (panSwitch == 1) {
          digitalWrite(panLeft, HIGH);
        }

        if (liftSwitch == 0) {
          digitalWrite(liftDown, HIGH);
        }
        if (liftSwitch == 1) {
          digitalWrite(liftUp, HIGH);
        }

        if (tiltSwitch == 0) {
          digitalWrite(tiltUp, HIGH);
        }
        if (tiltSwitch == 1) {
          digitalWrite(tiltDown, HIGH);
        }

        count++;
      }

      if (count >= mocoDistance) {

        //motor off
        if (swingSwitch == 0) {
          digitalWrite(swingLeft, LOW);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingRight, LOW);
        }
        if (panSwitch == 0) {
          digitalWrite(panRight, LOW);
        }
        if (panSwitch == 1) {
          digitalWrite(panLeft, LOW);
        }

        if (liftSwitch == 0) {
          digitalWrite(liftDown, LOW);
        }
        if (liftSwitch == 1) {
          digitalWrite(liftUp, LOW);
        }

        if (tiltSwitch == 0) {
          digitalWrite(tiltUp, LOW);
        }
        if (tiltSwitch == 1) {
          digitalWrite(tiltDown, LOW);
        }

        //motor on
        if (swingSwitch == 0) {
          digitalWrite(swingRight, HIGH);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingLeft, HIGH);
        }
        if (panSwitch == 0) {
          digitalWrite(panLeft, HIGH);
        }
        if (panSwitch == 1) {
          digitalWrite(panRight, HIGH);
        }

        if (liftSwitch == 0) {
          digitalWrite(liftUp, HIGH);
        }
        if (liftSwitch == 1) {
          digitalWrite(liftDown, HIGH);
        }

        if (tiltSwitch == 0) {
          digitalWrite(tiltDown, HIGH);
        }
        if (tiltSwitch == 1) {
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

      if (swingSwitch == 0) {
        digitalWrite(swingRight, HIGH);
      }
      if (swingSwitch == 1) {
        digitalWrite(swingLeft, HIGH);
      }
      if (panSwitch == 0) {
        digitalWrite(panLeft, HIGH);
      }
      if (panSwitch == 1) {
        digitalWrite(panRight, HIGH);
      }

      if (liftSwitch == 0) {
        digitalWrite(liftDown, HIGH);
      }
      if (liftSwitch == 1) {
        digitalWrite(liftUp, HIGH);
      }

      if (tiltSwitch == 0) {
        digitalWrite(tiltUp, HIGH);
      }
      if (tiltSwitch == 1) {
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
        if (swingSwitch == 0) {
          digitalWrite(swingRight, LOW);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingLeft, LOW);
        }
        if (panSwitch == 0) {
          digitalWrite(panLeft, LOW);
        }
        if (panSwitch == 1) {
          digitalWrite(panRight, LOW);
        }

        if (liftSwitch == 0) {
          digitalWrite(liftDown, LOW);
        }
        if (liftSwitch == 1) {
          digitalWrite(liftUp, LOW);
        }

        if (tiltSwitch == 0) {
          digitalWrite(tiltUp, LOW);
        }
        if (tiltSwitch == 1) {
          digitalWrite(tiltDown, LOW);
        }

        //motor on
        if (swingSwitch == 0) {
          digitalWrite(swingLeft, HIGH);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingRight, HIGH);
        }
        if (panSwitch == 0) {
          digitalWrite(panRight, HIGH);
        }
        if (panSwitch == 1) {
          digitalWrite(panLeft, HIGH);
        }

        if (liftSwitch == 0) {
          digitalWrite(liftUp, HIGH);
        }
        if (liftSwitch == 1) {
          digitalWrite(liftDown, HIGH);
        }

        if (tiltSwitch == 0) {
          digitalWrite(tiltDown, HIGH);
        }
        if (tiltSwitch == 1) {
          digitalWrite(tiltUp, HIGH);
        }

        count++;
      }

      if (count >= mocoDistance) {

        //motor OFF
        if (swingSwitch == 0) {
          digitalWrite(swingLeft, LOW);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingRight, LOW);
        }
        if (panSwitch == 0) {
          digitalWrite(panRight, LOW);
        }
        if (panSwitch == 1) {
          digitalWrite(panLeft, LOW);
        }

        if (liftSwitch == 0) {
          digitalWrite(liftUp, LOW);
        }
        if (liftSwitch == 1) {
          digitalWrite(liftDown, LOW);
        }

        if (tiltSwitch == 0) {
          digitalWrite(tiltDown, LOW);
        }
        if (tiltSwitch == 1) {
          digitalWrite(tiltUp, LOW);
        }

        //motor ON
        if (swingSwitch == 0) {
          digitalWrite(swingRight, HIGH);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingLeft, HIGH);
        }
        if (panSwitch == 0) {
          digitalWrite(panLeft, HIGH);
        }
        if (panSwitch == 1) {
          digitalWrite(panRight, HIGH);
        }

        if (liftSwitch == 0) {
          digitalWrite(liftDown, HIGH);
        }
        if (liftSwitch == 1) {
          digitalWrite(liftUp, HIGH);
        }

        if (tiltSwitch == 0) {
          digitalWrite(tiltUp, HIGH);
        }
        if (tiltSwitch == 1) {
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

      if (swingSwitch == 0) {
        digitalWrite(swingLeft, HIGH);
      }
      if (swingSwitch == 1) {
        digitalWrite(swingRight, HIGH);
      }
      if (panSwitch == 0) {
        digitalWrite(panRight, HIGH);
      }
      if (panSwitch == 1) {
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
        if (swingSwitch == 0) {
          digitalWrite(swingLeft, LOW);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingRight, LOW);
        }
        if (panSwitch == 0) {
          digitalWrite(panRight, LOW);
        }
        if (panSwitch == 1) {
          digitalWrite(panLeft, LOW);
        }
        //motor ON
        if (swingSwitch == 0) {
          digitalWrite(swingRight, HIGH);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingLeft, HIGH);
        }
        if (panSwitch == 0) {
          digitalWrite(panLeft, HIGH);
        }
        if (panSwitch == 1) {
          digitalWrite(panRight, HIGH);
        }

        count++;
      }

      if (count >= mocoDistance) {

        //motor OFF
        if (swingSwitch == 0) {
          digitalWrite(swingRight, LOW);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingLeft, LOW);
        }
        if (panSwitch == 0) {
          digitalWrite(panLeft, LOW);
        }
        if (panSwitch == 1) {
          digitalWrite(panRight, LOW);
        }

        //motor ON
        if (swingSwitch == 0) {
          digitalWrite(swingLeft, HIGH);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingRight, HIGH);
        }
        if (panSwitch == 0) {
          digitalWrite(panRight, HIGH);
        }
        if (panSwitch == 1) {
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

      if (liftSwitch == 0) {
        digitalWrite(liftUp, HIGH);
      }
      if (liftSwitch == 1) {
        digitalWrite(liftDown, HIGH);
      }

      if (tiltSwitch == 0) {
        digitalWrite(tiltDown, HIGH);
      }
      if (tiltSwitch == 1) {
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

        if (liftSwitch == 0) {
          digitalWrite(liftUp, LOW);
        }
        if (liftSwitch == 1) {
          digitalWrite(liftDown, LOW);
        }

        if (tiltSwitch == 0) {
          digitalWrite(tiltDown, LOW);
        }
        if (tiltSwitch == 1) {
          digitalWrite(tiltUp, LOW);
        }


        if (liftSwitch == 0) {
          digitalWrite(liftDown, HIGH);
        }
        if (liftSwitch == 1) {
          digitalWrite(liftUp, HIGH);
        }

        if (tiltSwitch == 0) {
          digitalWrite(tiltUp, HIGH);
        }
        if (tiltSwitch == 1) {
          digitalWrite(tiltDown, HIGH);
        }

        count++;
      }

      if (count >= mocoDistance) {

        if (liftSwitch == 0) {
          digitalWrite(liftDown, LOW);
        }
        if (liftSwitch == 1) {
          digitalWrite(liftUp, LOW);
        }

        if (tiltSwitch == 0) {
          digitalWrite(tiltUp, LOW);
        }
        if (tiltSwitch == 1) {
          digitalWrite(tiltDown, LOW);
        }


        if (liftSwitch == 0) {
          digitalWrite(liftUp, HIGH);
        }
        if (liftSwitch == 1) {
          digitalWrite(liftDown, HIGH);
        }

        if (tiltSwitch == 0) {
          digitalWrite(tiltDown, HIGH);
        }
        if (tiltSwitch == 1) {
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
      if (swingSwitch == 0) {
        digitalWrite(swingRight, HIGH);
      }
      if (swingSwitch == 1) {
        digitalWrite(swingLeft, HIGH);
      }
      if (panSwitch == 0) {
        digitalWrite(panLeft, HIGH);
      }
      if (panSwitch == 1) {
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


        if (swingSwitch == 0) {
          digitalWrite(swingRight, LOW);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingLeft, LOW);
        }
        if (panSwitch == 0) {
          digitalWrite(panLeft, LOW);
        }
        if (panSwitch == 1) {
          digitalWrite(panRight, LOW);
        }



        if (swingSwitch == 0) {
          digitalWrite(swingLeft, HIGH);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingRight, HIGH);
        }
        if (panSwitch == 0) {
          digitalWrite(panRight, HIGH);
        }
        if (panSwitch == 1) {
          digitalWrite(panLeft, HIGH);
        }

        count++;
      }

      if (count >= mocoDistance) {

        if (swingSwitch == 0) {
          digitalWrite(swingLeft, LOW);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingRight, LOW);
        }
        if (panSwitch == 0) {
          digitalWrite(panRight, LOW);
        }
        if (panSwitch == 1) {
          digitalWrite(panLeft, LOW);
        }


        if (swingSwitch == 0) {
          digitalWrite(swingRight, HIGH);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingLeft, HIGH);
        }
        if (panSwitch == 0) {
          digitalWrite(panLeft, HIGH);
        }
        if (panSwitch == 1) {
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

      if (liftSwitch == 0) {
        digitalWrite(liftDown, HIGH);
      }
      if (liftSwitch == 1) {
        digitalWrite(liftUp, HIGH);
      }

      if (tiltSwitch == 0) {
        digitalWrite(tiltUp, HIGH);
      }
      if (tiltSwitch == 1) {
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

        if (liftSwitch == 0) {
          digitalWrite(liftDown, LOW);
        }
        if (liftSwitch == 1) {
          digitalWrite(liftUp, LOW);
        }

        if (tiltSwitch == 0) {
          digitalWrite(tiltUp, LOW);
        }
        if (tiltSwitch == 1) {
          digitalWrite(tiltDown, LOW);
        }

        if (liftSwitch == 0) {
          digitalWrite(liftUp, HIGH);
        }
        if (liftSwitch == 1) {
          digitalWrite(liftDown, HIGH);
        }

        if (tiltSwitch == 0) {
          digitalWrite(tiltDown, HIGH);
        }
        if (tiltSwitch == 1) {
          digitalWrite(tiltUp, HIGH);
        }
        count++;
      }

      if (count >= mocoDistance) {

        if (liftSwitch == 0) {
          digitalWrite(liftUp, LOW);
        }
        if (liftSwitch == 1) {
          digitalWrite(liftDown, LOW);
        }

        if (tiltSwitch == 0) {
          digitalWrite(tiltDown, LOW);
        }
        if (tiltSwitch == 1) {
          digitalWrite(tiltUp, LOW);
        }


        if (liftSwitch == 0) {
          digitalWrite(liftDown, HIGH);
        }
        if (liftSwitch == 1) {
          digitalWrite(liftUp, HIGH);
        }

        if (tiltSwitch == 0) {
          digitalWrite(tiltUp, HIGH);
        }
        if (tiltSwitch == 1) {
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

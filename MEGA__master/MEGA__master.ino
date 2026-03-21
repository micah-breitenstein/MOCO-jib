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

int panleft = 46;
int panright = 48;
int panspeedup = 50;
int panspeeddown = 52;
int panstop = 0;
int speedstop = 0;

int panspeeduponly = 29; //lower
int panspeeddownonly = 45; //higher

int liftdown = 30;
int liftup = 31;
int liftspeedup = 32;
int liftspeeddown = 33;
int liftsolo = 0;

int tiltspeeduponly = 42; //lower val
int tiltspeeddownonly = 44; //higher val

int tiltdown = 34;
int tiltup = 36;
int tiltspeedup = 38;
int tiltspeeddown = 40;
int tiltstop = 0;

int focusleft = 47;
int focusright = 49;
int focusspeedup = 51;
int focusspeeddown = 53;

int swingInMotion = 0;
int liftinmotion = 0;

int leftStickXvalue;
int leftStickYvalue;
int rightStickXvalue;
int rightStickYvalue;

//timelapse variables
///////////////////
int timelapsemode = 0;
int intervalseconds = 15;
int interval;
int intervaldelay;
int stepdist = 100;
int trigger = 28;

//motion control variables
int bounce = 0;
int count = 0;
int mocodistance = 0;
int stage = 0;

int DIPSWITCH1 = 35;
int DIPSWITCH2 = 43;
int DIPSWITCH3 = 37;
int DIPSWITCH4 = 39;
int DIPSWITCH5 = 41;

int swingSwitch = 0;
int panswitch = 0;
int liftswitch = 0;
int tiltswitch = 0;
int focusswitch = 0;
int delaytime;

void setup() {

  //Serial.begin(57600);

  pinMode (swingLeft, OUTPUT);
  pinMode (swingRight, OUTPUT);
  pinMode (swingSpeedUp, OUTPUT);
  pinMode (swingSpeedDown, OUTPUT);
  pinMode (liftdown, OUTPUT);
  pinMode (liftup, OUTPUT);
  pinMode (liftspeedup, OUTPUT);
  pinMode (liftspeeddown, OUTPUT);
  pinMode (panleft, OUTPUT);
  pinMode (panright, OUTPUT);
  pinMode (panspeedup, OUTPUT);
  pinMode (panspeeddown, OUTPUT);
  pinMode (tiltdown, OUTPUT);
  pinMode (tiltup, OUTPUT);
  pinMode (tiltspeedup, OUTPUT);
  pinMode (tiltspeeddown, OUTPUT);
  pinMode (focusleft, OUTPUT);
  pinMode (focusright, OUTPUT);
  pinMode (focusspeedup, OUTPUT);
  pinMode (focusspeeddown, OUTPUT);

  pinMode(trigger, OUTPUT);
  interval = intervalseconds * 1000;

  pinMode (DIPSWITCH1, INPUT_PULLUP);
  pinMode (DIPSWITCH2, INPUT_PULLUP);
  pinMode (DIPSWITCH3, INPUT_PULLUP);
  pinMode (DIPSWITCH4, INPUT_PULLUP);
  pinMode (DIPSWITCH5, INPUT_PULLUP);

  pinMode (tiltspeeduponly, OUTPUT);
  pinMode (tiltspeeddownonly, OUTPUT);

  pinMode (panspeeduponly, OUTPUT);
  pinMode (panspeeddownonly, OUTPUT);



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

    swingSwitch = digitalRead(DIPSWITCH1);
    panswitch = digitalRead(DIPSWITCH2);
    liftswitch = digitalRead(DIPSWITCH3);
    tiltswitch = digitalRead(DIPSWITCH4);
    focusswitch =  digitalRead(DIPSWITCH5);

    rightStickYvalue = ps2x.Analog(PSS_RY);
    rightStickXvalue = ps2x.Analog(PSS_RX);
    leftStickYvalue  = ps2x.Analog(PSS_LY);
    leftStickXvalue  = ps2x.Analog(PSS_LX);


    //////////////R1 and R2 Buttons
    if (ps2x.Button(PSB_R1)) {
      digitalWrite(liftspeedup, HIGH);
      digitalWrite(tiltspeedup, HIGH);
    }
    if (ps2x.ButtonReleased(PSB_R1)) {
      digitalWrite(liftspeedup, LOW);
      digitalWrite(tiltspeedup, LOW);
    }

    if (ps2x.Button(PSB_R2)) {
      digitalWrite(liftspeeddown, HIGH);
      digitalWrite(tiltspeeddown, HIGH);
    }

    if (ps2x.ButtonReleased(PSB_R2)) {
      digitalWrite(liftspeeddown, LOW);
      digitalWrite(tiltspeeddown, LOW);
    }

    ///////////L1 ad L2 Buttons
    if (ps2x.Button(PSB_L1)) {
      digitalWrite(panspeedup, HIGH);
      digitalWrite(swingSpeedUp, HIGH);
    }
    if (ps2x.ButtonReleased(PSB_L1)) {
      digitalWrite(panspeedup, LOW);
      digitalWrite(swingSpeedUp, LOW);
    }

    if (ps2x.Button(PSB_L2)) {
      digitalWrite(panspeeddown, HIGH);
      digitalWrite(swingSpeedDown, HIGH);
    }
    if (ps2x.ButtonReleased(PSB_L2)) {
      digitalWrite(panspeeddown, LOW);
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

    ///////////swingLeft panright
    /////////////////////////////////
    if (swingSoloMode == 0 && ps2x.Button(PSB_PAD_LEFT)) {
      swingInMotion = 1;
      if (swingSwitch == 0) {
        digitalWrite(swingLeft, HIGH);
      }
      if (swingSwitch == 1) {
        digitalWrite(swingRight, HIGH);
      }
      if (panswitch == 0) {
        digitalWrite(panright, HIGH);
      }
      if (panswitch == 1) {
        digitalWrite(panleft, HIGH);
      }
    }

    if (swingSoloMode == 0 && ps2x.ButtonReleased(PSB_PAD_LEFT)) {
      digitalWrite(swingLeft, LOW);
      digitalWrite (panright, LOW);
      digitalWrite(swingRight, LOW);
      digitalWrite (panleft, LOW);
      swingInMotion = 0;
    }


    ///////////swingRight panleft
    //////////////////////////////
    if (swingSoloMode == 0 && ps2x.Button(PSB_PAD_RIGHT)) {
      swingInMotion = 1;
      if (swingSwitch == 0) {
        digitalWrite(swingRight, HIGH);
      }
      if (swingSwitch == 1) {
        digitalWrite(swingLeft, HIGH);
      }
      if (panswitch == 0) {
        digitalWrite(panleft, HIGH);
      }
      if (panswitch == 1) {
        digitalWrite(panright, HIGH);
      }
    }

    if (swingSoloMode == 0 && ps2x.ButtonReleased(PSB_PAD_RIGHT)) {
      digitalWrite(swingRight, LOW);
      digitalWrite (panleft, LOW);
      digitalWrite(swingLeft, LOW);
      digitalWrite (panright, LOW);
      swingInMotion = 0;
    }

    //should all these below be within 1 if statement for swingmotion?
    if (swingInMotion == 1 && rightStickXvalue == 0) {
      digitalWrite(panspeeddownonly, HIGH);
      Serial.println("panspeeddownonly");
      panstop = 2;
    }

    if (swingInMotion == 1 && rightStickXvalue == 255) {
      digitalWrite(panspeeduponly, HIGH);
      Serial.println("panspeeduponly");
      panstop = 2;
    }

    if (panstop == 2 && rightStickXvalue == 128  ) {
      digitalWrite(panspeeduponly, LOW);
      digitalWrite(panspeeddownonly, LOW);
      panstop = 0;
    }

    /////////////////////////////
    //2nd AXIS (CAMERA PAN)
    ////////////////////////////

    if (swingInMotion == 0 &&  rightStickXvalue == 0) {
      Serial.println("panleftnonly with top speed");

      digitalWrite(panspeeduponly, HIGH); //signal to nano to use top speed
      digitalWrite(panspeeddownonly, HIGH);//signal to nano to use top speed

      if (panswitch == 0) {
        digitalWrite(panleft, HIGH);
      }
      if (panswitch == 1) {
        digitalWrite(panright, HIGH);
      }
      panstop = 1;
    }

    if (swingInMotion == 0 && rightStickXvalue == 255) {
      Serial.println("panrightonly with top speed");
      digitalWrite(panspeeduponly, HIGH); //signal to nano to use top speed
      digitalWrite(panspeeddownonly, HIGH);//signal to nano to use top speed

      if (panswitch == 0) {
        digitalWrite(panright, HIGH);
      }
      if (panswitch == 1) {
        digitalWrite(panleft, HIGH);
      }
      panstop = 1;
    }

    if (swingInMotion == 0 && panstop == 1 && rightStickXvalue == 128) {
      digitalWrite(panleft, LOW);
      digitalWrite(panright, LOW);
      digitalWrite(panspeeduponly, LOW);
      digitalWrite(panspeeddownonly, LOW);
      panstop = 0;
    }


    /////////////////////////////
    //3rd AXIS (BOOM LIFT)
    ////////////////////////////

    //lift UP (no tilt)
    if (ps2x.Button(PSB_SELECT) && ps2x.Button(PSB_PAD_UP)) {
      if (liftswitch == 0) {
        digitalWrite(liftup, HIGH);
      }
      if (liftswitch == 1) {
        digitalWrite(liftdown, HIGH);
      }
      liftsolo = 1;
    }

    if (liftsolo == 1 && ps2x.ButtonReleased(PSB_PAD_UP)) {
      digitalWrite(liftup, LOW);
      digitalWrite(liftdown, LOW);
      liftsolo = 0;
    }

    //lift DOWN (no tilt)
    if (ps2x.Button(PSB_SELECT) && ps2x.Button(PSB_PAD_DOWN)) {
      if (liftswitch == 0) {
        digitalWrite(liftdown, HIGH);
      }
      if (liftswitch == 1) {
        digitalWrite(liftup, HIGH);
      }
      liftsolo = 1;
    }

    if (liftsolo == 1  && ps2x.ButtonReleased(PSB_PAD_DOWN)) {
      digitalWrite(liftdown, LOW);
      digitalWrite(liftup, LOW);
      liftsolo = 0;
    }


    //lift UP tilt DOWN
    ///////////////////

    if (liftsolo == 0 && ps2x.Button(PSB_PAD_UP)) {
      liftinmotion = 1;
      if (liftswitch == 0) {
        digitalWrite(liftup, HIGH);
      }
      if (liftswitch == 1) {
        digitalWrite(liftdown, HIGH);
      }

      if (tiltswitch == 0) {
        digitalWrite(tiltdown, HIGH);
      }
      if (tiltswitch == 1) {
        digitalWrite(tiltup, HIGH);
      }
    }

    if (liftsolo == 0 && ps2x.ButtonReleased(PSB_PAD_UP)) {
      digitalWrite(liftup, LOW);
      digitalWrite(tiltdown, LOW);
      digitalWrite(liftdown, LOW);
      digitalWrite(tiltup, LOW);
      liftinmotion = 0;
    }

    //lift DOWN tilt UP
    ///////////////////
    if (liftsolo == 0 && ps2x.Button(PSB_PAD_DOWN)) {
      liftinmotion = 1;

      if (liftswitch == 0) {
        digitalWrite(liftdown, HIGH);
      }
      if (liftswitch == 1) {
        digitalWrite(liftup, HIGH);
      }

      if (tiltswitch == 0) {
        digitalWrite(tiltup, HIGH);
      }
      if (tiltswitch == 1) {
        digitalWrite(tiltdown, HIGH);
      }
    }

    if (liftsolo == 0 && ps2x.ButtonReleased(PSB_PAD_DOWN)) {
      digitalWrite(liftup, LOW);
      digitalWrite(tiltdown, LOW);
      digitalWrite(liftdown, LOW);
      digitalWrite(tiltup, LOW);
      liftinmotion = 0;
    }

    //should this all be inn 1 liftinmotion if statement?

    if (liftinmotion == 1 && rightStickYvalue == 255) {
      Serial.println("tiltspeeddownonly");
      digitalWrite(tiltspeeddownonly, HIGH);
      tiltstop = 2;
    }

    if (liftinmotion == 1 && rightStickYvalue == 0) {
      Serial.println("tiltspeedupnonly");
      digitalWrite(tiltspeeduponly, HIGH);
      tiltstop = 2;
    }
    //should below also be a liftinmotion?
    if (tiltstop == 2 && rightStickYvalue == 128  ) {
      digitalWrite(tiltspeeduponly, LOW);
      digitalWrite(tiltspeeddownonly, LOW);
      tiltstop = 0;
    }

    /////////////////////////////
    //2nd AXIS (CAMERA tilt)
    ////////////////////////////

    if (liftinmotion == 0  && rightStickYvalue == 255) {
      Serial.println("tiltdownonly");
      //signal to nano to use top speed
      digitalWrite(tiltspeeduponly, HIGH); //signal to nano to use top speed
      digitalWrite(tiltspeeddownonly, HIGH); //signal to nano to use top speed

      if (tiltswitch == 0) {
        digitalWrite(tiltdown, HIGH);
      }
      if (tiltswitch == 1) {
        digitalWrite(tiltup, HIGH);
      }
      tiltstop = 1;
    }

    if (liftinmotion == 0 && rightStickYvalue == 0) {
      Serial.println("tiltuponly");

      digitalWrite(tiltspeeduponly, HIGH); //signal to nano to use top speed
      digitalWrite(tiltspeeddownonly, HIGH); //signal to nano to use top speed
      if (tiltswitch == 0) {
        digitalWrite(tiltup, HIGH);
      }
      if (tiltswitch == 1) {
        digitalWrite(tiltdown, HIGH);
      }
      tiltstop = 1;
    }

    if (liftinmotion == 0 && tiltstop == 1 && rightStickYvalue == 128) {
      digitalWrite(tiltup, LOW);
      digitalWrite(tiltdown, LOW);
      digitalWrite(tiltspeeduponly, LOW);
      digitalWrite(tiltspeeddownonly, LOW);
      tiltstop = 0;
    }


    /////////////////////////////
    //5th AXIS (Camera focus)
    ////////////////////////////

    if (ps2x.Button(PSB_TRIANGLE)) {
      if (focusswitch == 0) {
        digitalWrite(focusleft, HIGH);
      }
      if (focusswitch == 1) {
        digitalWrite(focusright, HIGH);
      }

    }
    if (ps2x.ButtonReleased(PSB_TRIANGLE)) {
      digitalWrite(focusleft, LOW);
      digitalWrite(focusright, LOW);
    }

    if (ps2x.Button(PSB_CROSS)) {
      if (focusswitch == 0) {
        digitalWrite(focusright, HIGH);
      }
      if (focusswitch == 1) {
        digitalWrite(focusleft, HIGH);
      }
    }

    if (ps2x.ButtonReleased(PSB_CROSS)) {
      digitalWrite(focusright, LOW);
      digitalWrite(focusleft, LOW);
    }


    if (ps2x.Button(PSB_SQUARE)) {
      digitalWrite(focusspeeddown, HIGH);
    }
    if (ps2x.ButtonReleased(PSB_SQUARE)) {
      digitalWrite(focusspeeddown, LOW);
    }

    if (ps2x.Button(PSB_CIRCLE)) {
      digitalWrite(focusspeedup, HIGH);
    }
    if (ps2x.ButtonReleased(PSB_CIRCLE)) {
      digitalWrite(focusspeedup, LOW);
    }



    ////////////////////////////////////////////////////
    /////////////////////Timelapse/////////////////////
    //////////////////////////////////////////////////


    ///////swing left boom down
    ///////////////////////////////////

    if  (timelapsemode == 0 && leftStickXvalue < 123 && leftStickYvalue > 133 && ps2x.ButtonReleased(PSB_SELECT)) {
      timelapsemode = 1; //swing left boom down
    }

    if (timelapsemode == 1) {
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
      if (panswitch == 0) {
        digitalWrite(panright, HIGH);
      }
      if (panswitch == 1) {
        digitalWrite(panleft, HIGH);
      }

      if (liftswitch == 0) {
        digitalWrite(liftdown, HIGH);
      }
      if (liftswitch == 1) {
        digitalWrite(liftup, HIGH);
      }

      if (tiltswitch == 0) {
        digitalWrite(tiltup, HIGH);
      }
      if (tiltswitch == 1) {
        digitalWrite(tiltdown, HIGH);
      }

      delay(stepdist);

      Serial.println("turning off timelapse 1 now");
      digitalWrite(swingLeft, LOW);
      digitalWrite(swingRight, LOW);
      digitalWrite(panright, LOW);
      digitalWrite(panleft, LOW);
      digitalWrite(liftdown, LOW);
      digitalWrite(liftup, LOW);
      digitalWrite(tiltup, LOW);
      digitalWrite(tiltdown, LOW);
    }

    ///////swing LEFT and lift UP
    ///////////////////////////////////
    if (timelapsemode == 0 && leftStickXvalue < 123 && leftStickYvalue < 123 && ps2x.ButtonReleased(PSB_SELECT)) {
      timelapsemode = 2; //swing left boom up
    }

    if (timelapsemode == 2) {
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
      if (panswitch == 0) {
        digitalWrite(panright, HIGH);
      }
      if (panswitch == 1) {
        digitalWrite(panleft, HIGH);
      }

      if (liftswitch == 0) {
        digitalWrite(liftup, HIGH);
      }
      if (liftswitch == 1) {
        digitalWrite(liftdown, HIGH);
      }

      if (tiltswitch == 0) {
        digitalWrite(tiltdown, HIGH);
      }
      if (tiltswitch == 1) {
        digitalWrite(tiltup, HIGH);
      }


      delay(stepdist);
      digitalWrite(swingLeft, LOW);
      digitalWrite(swingRight, LOW);
      digitalWrite(panright, LOW);
      digitalWrite(panleft, LOW);
      digitalWrite(liftdown, LOW);
      digitalWrite(liftup, LOW);
      digitalWrite(tiltup, LOW);
      digitalWrite(tiltdown, LOW);
    }

    ///////swing right and boomup
    ///////////////////////////////////
    if (timelapsemode == 0 && leftStickXvalue > 133 && leftStickYvalue < 123 && ps2x.ButtonReleased(PSB_SELECT)) {
      timelapsemode = 3;  //swing right and boom up
    }

    if (timelapsemode == 3) {
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
      if (panswitch == 0) {
        digitalWrite(panleft, HIGH);
      }
      if (panswitch == 1) {
        digitalWrite(panright, HIGH);
      }

      if (liftswitch == 0) {
        digitalWrite(liftup, HIGH);
      }
      if (liftswitch == 1) {
        digitalWrite(liftdown, HIGH);
      }

      if (tiltswitch == 0) {
        digitalWrite(tiltdown, HIGH);
      }
      if (tiltswitch == 1) {
        digitalWrite(tiltup, HIGH);
      }



      delay(stepdist);

      digitalWrite(swingLeft, LOW);
      digitalWrite(swingRight, LOW);
      digitalWrite(panright, LOW);
      digitalWrite(panleft, LOW);
      digitalWrite(liftdown, LOW);
      digitalWrite(liftup, LOW);
      digitalWrite(tiltup, LOW);
      digitalWrite(tiltdown, LOW);
    }


    ///////swing right and boomdown
    ///////////////////////////////////
    if (timelapsemode == 0 && leftStickXvalue > 133 && leftStickYvalue > 133 && ps2x.ButtonReleased(PSB_SELECT)) {
      timelapsemode = 4; //swing right and boomdown
    }

    if (timelapsemode == 4) {
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
      if (panswitch == 0) {
        digitalWrite(panleft, HIGH);
      }
      if (panswitch == 1) {
        digitalWrite(panright, HIGH);
      }

      if (liftswitch == 0) {
        digitalWrite(liftdown, HIGH);
      }
      if (liftswitch == 1) {
        digitalWrite(liftup, HIGH);
      }

      if (tiltswitch == 0) {
        digitalWrite(tiltup, HIGH);
      }
      if (tiltswitch == 1) {
        digitalWrite(tiltdown, HIGH);
      }


      delay(stepdist);
      digitalWrite(swingLeft, LOW);
      digitalWrite(swingRight, LOW);
      digitalWrite(panright, LOW);
      digitalWrite(panleft, LOW);
      digitalWrite(liftdown, LOW);
      digitalWrite(liftup, LOW);
      digitalWrite(tiltup, LOW);
      digitalWrite(tiltdown, LOW);
    }

    ///////swing left
    ///////////////////////////////////

    if  (timelapsemode == 0 && leftStickXvalue == 0 && ps2x.ButtonReleased(PSB_SELECT)) {
      timelapsemode = 5; //swing left
    }

    if (timelapsemode == 5) {
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
      if (panswitch == 0) {
        digitalWrite(panright, HIGH);
      }
      if (panswitch == 1) {
        digitalWrite(panleft, HIGH);
      }

      delay(stepdist);
      digitalWrite(swingLeft, LOW);
      digitalWrite(swingRight, LOW);
      digitalWrite(panright, LOW);
      digitalWrite(panleft, LOW);

    }


    ///////lift up
    ///////////////////////////////////
    if (timelapsemode == 0 && leftStickYvalue == 0 && ps2x.ButtonReleased(PSB_SELECT)) {
      timelapsemode = 6; // boom up
    }

    if (timelapsemode == 6) {
      Serial.println("timelapse mode 6");
      digitalWrite(trigger, LOW);
      delay (interval / 2);
      digitalWrite(trigger, HIGH);
      delay (interval / 2);

      if (liftswitch == 0) {
        digitalWrite(liftup, HIGH);
      }
      if (liftswitch == 1) {
        digitalWrite(liftdown, HIGH);
      }

      if (tiltswitch == 0) {
        digitalWrite(tiltdown, HIGH);
      }
      if (tiltswitch == 1) {
        digitalWrite(tiltup, HIGH);
      }

      delay(stepdist);
      digitalWrite(liftdown, LOW);
      digitalWrite(liftup, LOW);
      digitalWrite(tiltup, LOW);
      digitalWrite(tiltdown, LOW);
    }


    ///////swing right
    ///////////////////////////////////
    if (timelapsemode == 0 && leftStickXvalue == 255 && ps2x.ButtonReleased(PSB_SELECT)) {
      timelapsemode = 7;  //swing right
    }

    if (timelapsemode == 7) {
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
      if (panswitch == 0) {
        digitalWrite(panleft, HIGH);
      }
      if (panswitch == 1) {
        digitalWrite(panright, HIGH);
      }

      delay(stepdist);
      digitalWrite(swingLeft, LOW);
      digitalWrite(swingRight, LOW);
      digitalWrite(panright, LOW);
      digitalWrite(panleft, LOW);

    }


    /////lift DOWN
    ///////////////
    if (timelapsemode == 0 && leftStickYvalue == 255 && ps2x.ButtonReleased(PSB_SELECT)) {
      timelapsemode = 8;
    }

    if (timelapsemode == 8) {
      Serial.println("timelapse mode 8");
      digitalWrite(trigger, LOW);
      delay (interval / 2);
      digitalWrite(trigger, HIGH);
      delay (interval / 2);


      if (liftswitch == 0) {
        digitalWrite(liftdown, HIGH);
      }
      if (liftswitch == 1) {
        digitalWrite(liftup, HIGH);
      }

      if (tiltswitch == 0) {
        digitalWrite(tiltup, HIGH);
      }
      if (tiltswitch == 1) {
        digitalWrite(tiltdown, HIGH);
      }

      delay(stepdist);
      digitalWrite(liftdown, LOW);
      digitalWrite(liftup, LOW);
      digitalWrite(tiltup, LOW);
      digitalWrite(tiltdown, LOW);
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
      if (panswitch == 0) {
        digitalWrite(panright, HIGH);
      }
      if (panswitch == 1) {
        digitalWrite(panleft, HIGH);
      }

      if (liftswitch == 0) {
        digitalWrite(liftdown, HIGH);
      }
      if (liftswitch == 1) {
        digitalWrite(liftup, HIGH);
      }

      if (tiltswitch == 0) {
        digitalWrite(tiltup, HIGH);
      }
      if (tiltswitch == 1) {
        digitalWrite(tiltdown, HIGH);
      }
      count++;

    }
    if (bounce == 1 && stage == 0 && ps2x.ButtonReleased(PSB_L3)) {

      digitalWrite(swingLeft, LOW);
      digitalWrite(swingRight, LOW);
      digitalWrite(panright, LOW);
      digitalWrite(panleft, LOW);
      digitalWrite(liftdown, LOW);
      digitalWrite(liftup, LOW);
      digitalWrite(tiltup, LOW);
      digitalWrite(tiltdown, LOW);

      mocodistance = count;
      count = 0;
      stage = 1;
    }

    if (bounce == 1 && stage == 1) { //end point
      if (count <= mocodistance) {

        //turn off motors
        if (swingSwitch == 0) {
          digitalWrite(swingLeft, LOW);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingRight, LOW);
        }
        if (panswitch == 0) {
          digitalWrite(panright, LOW);
        }
        if (panswitch == 1) {
          digitalWrite(panleft, LOW);
        }

        if (liftswitch == 0) {
          digitalWrite(liftdown, LOW);
        }
        if (liftswitch == 1) {
          digitalWrite(liftup, LOW);
        }

        if (tiltswitch == 0) {
          digitalWrite(tiltup, LOW);
        }
        if (tiltswitch == 1) {
          digitalWrite(tiltdown, LOW);
        }


        //turn on motors

        if (swingSwitch == 0) {
          digitalWrite(swingRight, HIGH);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingLeft, HIGH);
        }
        if (panswitch == 0) {
          digitalWrite(panleft, HIGH);
        }
        if (panswitch == 1) {
          digitalWrite(panright, HIGH);
        }

        if (liftswitch == 0) {
          digitalWrite(liftup, HIGH);
        }
        if (liftswitch == 1) {
          digitalWrite(liftdown, HIGH);
        }

        if (tiltswitch == 0) {
          digitalWrite(tiltdown, HIGH);
        }
        if (tiltswitch == 1) {
          digitalWrite(tiltup, HIGH);
        }
        count++;
      }

      if (count >= mocodistance) {//starting point

        if (swingSwitch == 0) {
          digitalWrite(swingRight, LOW);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingLeft, LOW);
        }
        if (panswitch == 0) {
          digitalWrite(panleft, LOW);
        }
        if (panswitch == 1) {
          digitalWrite(panright, LOW);
        }

        if (liftswitch == 0) {
          digitalWrite(liftup, LOW);
        }
        if (liftswitch == 1) {
          digitalWrite(liftdown, LOW);
        }

        if (tiltswitch == 0) {
          digitalWrite(tiltdown, LOW);
        }
        if (tiltswitch == 1) {
          digitalWrite(tiltup, LOW);
        }

        //turn motors on

        if (swingSwitch == 0) {
          digitalWrite(swingLeft, HIGH);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingRight, HIGH);
        }
        if (panswitch == 0) {
          digitalWrite(panright, HIGH);
        }
        if (panswitch == 1) {
          digitalWrite(panleft, HIGH);
        }

        if (liftswitch == 0) {
          digitalWrite(liftdown, HIGH);
        }
        if (liftswitch == 1) {
          digitalWrite(liftup, HIGH);
        }

        if (tiltswitch == 0) {
          digitalWrite(tiltup, HIGH);
        }
        if (tiltswitch == 1) {
          digitalWrite(tiltdown, HIGH);
        }
        count++;
      }

      if (count >= mocodistance * 2) {
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
      if (panswitch == 0) {
        digitalWrite(panright, HIGH);
      }
      if (panswitch == 1) {
        digitalWrite(panleft, HIGH);
      }

      if (liftswitch == 0) {
        digitalWrite(liftup, HIGH);
      }
      if (liftswitch == 1) {
        digitalWrite(liftdown, HIGH);
      }

      if (tiltswitch == 0) {
        digitalWrite(tiltdown, HIGH);
      }
      if (tiltswitch == 1) {
        digitalWrite(tiltup, HIGH);
      }
      count++;

    }
    if (bounce == 2 && stage == 0 && ps2x.ButtonReleased(PSB_L3)) {
      digitalWrite(swingLeft, LOW);
      digitalWrite(swingRight, LOW);
      digitalWrite(panright, LOW);
      digitalWrite(panleft, LOW);
      digitalWrite(liftdown, LOW);
      digitalWrite(liftup, LOW);
      digitalWrite(tiltup, LOW);
      digitalWrite(tiltdown, LOW);

      mocodistance = count;
      count = 0;
      stage = 1;
    }

    if (bounce == 2 && stage == 1) {
      if (count <= mocodistance) {

        //turn motors off

        if (swingSwitch == 0) {
          digitalWrite(swingLeft, LOW);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingRight, LOW);
        }
        if (panswitch == 0) {
          digitalWrite(panright, LOW);
        }
        if (panswitch == 1) {
          digitalWrite(panleft, LOW);
        }

        if (liftswitch == 0) {
          digitalWrite(liftup, LOW);
        }
        if (liftswitch == 1) {
          digitalWrite(liftdown, LOW);
        }

        if (tiltswitch == 0) {
          digitalWrite(tiltdown, LOW);
        }
        if (tiltswitch == 1) {
          digitalWrite(tiltup, LOW);
        }


        //turn motors on

        if (swingSwitch == 0) {
          digitalWrite(swingRight, HIGH);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingLeft, HIGH);
        }
        if (panswitch == 0) {
          digitalWrite(panleft, HIGH);
        }
        if (panswitch == 1) {
          digitalWrite(panright, HIGH);
        }

        if (liftswitch == 0) {
          digitalWrite(liftdown, HIGH);
        }
        if (liftswitch == 1) {
          digitalWrite(liftup, HIGH);
        }

        if (tiltswitch == 0) {
          digitalWrite(tiltup, HIGH);
        }
        if (tiltswitch == 1) {
          digitalWrite(tiltdown, HIGH);
        }
        count++;
      }

      if (count >= mocodistance) {

        //turn motors off
        if (swingSwitch == 0) {
          digitalWrite(swingRight, LOW);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingLeft, LOW);
        }
        if (panswitch == 0) {
          digitalWrite(panleft, LOW);
        }
        if (panswitch == 1) {
          digitalWrite(panright, LOW);
        }

        if (liftswitch == 0) {
          digitalWrite(liftdown, LOW);
        }
        if (liftswitch == 1) {
          digitalWrite(liftup, LOW);
        }

        if (tiltswitch == 0) {
          digitalWrite(tiltup, LOW);
        }
        if (tiltswitch == 1) {
          digitalWrite(tiltdown, LOW);
        }

        //turn motors on
        if (swingSwitch == 0) {
          digitalWrite(swingLeft, HIGH);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingRight, HIGH);
        }
        if (panswitch == 0) {
          digitalWrite(panright, HIGH);
        }
        if (panswitch == 1) {
          digitalWrite(panleft, HIGH);
        }

        if (liftswitch == 0) {
          digitalWrite(liftup, HIGH);
        }
        if (liftswitch == 1) {
          digitalWrite(liftdown, HIGH);
        }

        if (tiltswitch == 0) {
          digitalWrite(tiltdown, HIGH);
        }
        if (tiltswitch == 1) {
          digitalWrite(tiltup, HIGH);
        }
        count++;
      }

      if (count >= mocodistance * 2) {
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
      if (panswitch == 0) {
        digitalWrite(panleft, HIGH);
      }
      if (panswitch == 1) {
        digitalWrite(panright, HIGH);
      }

      if (liftswitch == 0) {
        digitalWrite(liftup, HIGH);
      }
      if (liftswitch == 1) {
        digitalWrite(liftdown, HIGH);
      }

      if (tiltswitch == 0) {
        digitalWrite(tiltdown, HIGH);
      }
      if (tiltswitch == 1) {
        digitalWrite(tiltup, HIGH);
      }

      count++;
    }

    if (bounce == 3 && stage == 0 && ps2x.ButtonReleased(PSB_L3)) {

      digitalWrite(swingLeft, LOW);
      digitalWrite(swingRight, LOW);
      digitalWrite(panright, LOW);
      digitalWrite(panleft, LOW);
      digitalWrite(liftdown, LOW);
      digitalWrite(liftup, LOW);
      digitalWrite(tiltup, LOW);
      digitalWrite(tiltdown, LOW);

      mocodistance = count;
      count = 0;
      stage = 1;
    }

    if (bounce == 3 && stage == 1) {

      if (count <= mocodistance) {

        // motor off
        if (swingSwitch == 0) {
          digitalWrite(swingRight, LOW);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingLeft, LOW);
        }
        if (panswitch == 0) {
          digitalWrite(panleft, LOW);
        }
        if (panswitch == 1) {
          digitalWrite(panright, LOW);
        }

        if (liftswitch == 0) {
          digitalWrite(liftup, LOW);
        }
        if (liftswitch == 1) {
          digitalWrite(liftdown, LOW);
        }

        if (tiltswitch == 0) {
          digitalWrite(tiltdown, LOW);
        }
        if (tiltswitch == 1) {
          digitalWrite(tiltup, LOW);
        }


        //motor on
        if (swingSwitch == 0) {
          digitalWrite(swingLeft, HIGH);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingRight, HIGH);
        }
        if (panswitch == 0) {
          digitalWrite(panright, HIGH);
        }
        if (panswitch == 1) {
          digitalWrite(panleft, HIGH);
        }

        if (liftswitch == 0) {
          digitalWrite(liftdown, HIGH);
        }
        if (liftswitch == 1) {
          digitalWrite(liftup, HIGH);
        }

        if (tiltswitch == 0) {
          digitalWrite(tiltup, HIGH);
        }
        if (tiltswitch == 1) {
          digitalWrite(tiltdown, HIGH);
        }

        count++;
      }

      if (count >= mocodistance) {

        //motor off
        if (swingSwitch == 0) {
          digitalWrite(swingLeft, LOW);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingRight, LOW);
        }
        if (panswitch == 0) {
          digitalWrite(panright, LOW);
        }
        if (panswitch == 1) {
          digitalWrite(panleft, LOW);
        }

        if (liftswitch == 0) {
          digitalWrite(liftdown, LOW);
        }
        if (liftswitch == 1) {
          digitalWrite(liftup, LOW);
        }

        if (tiltswitch == 0) {
          digitalWrite(tiltup, LOW);
        }
        if (tiltswitch == 1) {
          digitalWrite(tiltdown, LOW);
        }

        //motor on
        if (swingSwitch == 0) {
          digitalWrite(swingRight, HIGH);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingLeft, HIGH);
        }
        if (panswitch == 0) {
          digitalWrite(panleft, HIGH);
        }
        if (panswitch == 1) {
          digitalWrite(panright, HIGH);
        }

        if (liftswitch == 0) {
          digitalWrite(liftup, HIGH);
        }
        if (liftswitch == 1) {
          digitalWrite(liftdown, HIGH);
        }

        if (tiltswitch == 0) {
          digitalWrite(tiltdown, HIGH);
        }
        if (tiltswitch == 1) {
          digitalWrite(tiltup, HIGH);
        }

        count++;
      }
      if (count >= mocodistance * 2) {
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
      if (panswitch == 0) {
        digitalWrite(panleft, HIGH);
      }
      if (panswitch == 1) {
        digitalWrite(panright, HIGH);
      }

      if (liftswitch == 0) {
        digitalWrite(liftdown, HIGH);
      }
      if (liftswitch == 1) {
        digitalWrite(liftup, HIGH);
      }

      if (tiltswitch == 0) {
        digitalWrite(tiltup, HIGH);
      }
      if (tiltswitch == 1) {
        digitalWrite(tiltdown, HIGH);
      }

      count++;

    }

    if (bounce == 4 && stage == 0 && ps2x.ButtonReleased(PSB_L3)) {

      digitalWrite(swingLeft, LOW);
      digitalWrite(swingRight, LOW);
      digitalWrite(panright, LOW);
      digitalWrite(panleft, LOW);
      digitalWrite(liftdown, LOW);
      digitalWrite(liftup, LOW);
      digitalWrite(tiltup, LOW);
      digitalWrite(tiltdown, LOW);
      mocodistance = count;
      count = 0;
      stage = 1;

    }

    if (bounce == 4 && stage == 1) {
      if (count <= mocodistance) {

        //motor off
        if (swingSwitch == 0) {
          digitalWrite(swingRight, LOW);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingLeft, LOW);
        }
        if (panswitch == 0) {
          digitalWrite(panleft, LOW);
        }
        if (panswitch == 1) {
          digitalWrite(panright, LOW);
        }

        if (liftswitch == 0) {
          digitalWrite(liftdown, LOW);
        }
        if (liftswitch == 1) {
          digitalWrite(liftup, LOW);
        }

        if (tiltswitch == 0) {
          digitalWrite(tiltup, LOW);
        }
        if (tiltswitch == 1) {
          digitalWrite(tiltdown, LOW);
        }

        //motor on
        if (swingSwitch == 0) {
          digitalWrite(swingLeft, HIGH);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingRight, HIGH);
        }
        if (panswitch == 0) {
          digitalWrite(panright, HIGH);
        }
        if (panswitch == 1) {
          digitalWrite(panleft, HIGH);
        }

        if (liftswitch == 0) {
          digitalWrite(liftup, HIGH);
        }
        if (liftswitch == 1) {
          digitalWrite(liftdown, HIGH);
        }

        if (tiltswitch == 0) {
          digitalWrite(tiltdown, HIGH);
        }
        if (tiltswitch == 1) {
          digitalWrite(tiltup, HIGH);
        }

        count++;
      }

      if (count >= mocodistance) {

        //motor OFF
        if (swingSwitch == 0) {
          digitalWrite(swingLeft, LOW);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingRight, LOW);
        }
        if (panswitch == 0) {
          digitalWrite(panright, LOW);
        }
        if (panswitch == 1) {
          digitalWrite(panleft, LOW);
        }

        if (liftswitch == 0) {
          digitalWrite(liftup, LOW);
        }
        if (liftswitch == 1) {
          digitalWrite(liftdown, LOW);
        }

        if (tiltswitch == 0) {
          digitalWrite(tiltdown, LOW);
        }
        if (tiltswitch == 1) {
          digitalWrite(tiltup, LOW);
        }

        //motor ON
        if (swingSwitch == 0) {
          digitalWrite(swingRight, HIGH);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingLeft, HIGH);
        }
        if (panswitch == 0) {
          digitalWrite(panleft, HIGH);
        }
        if (panswitch == 1) {
          digitalWrite(panright, HIGH);
        }

        if (liftswitch == 0) {
          digitalWrite(liftdown, HIGH);
        }
        if (liftswitch == 1) {
          digitalWrite(liftup, HIGH);
        }

        if (tiltswitch == 0) {
          digitalWrite(tiltup, HIGH);
        }
        if (tiltswitch == 1) {
          digitalWrite(tiltdown, HIGH);
        }

        count++;
      }

      if (count >= mocodistance * 2) {
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
      if (panswitch == 0) {
        digitalWrite(panright, HIGH);
      }
      if (panswitch == 1) {
        digitalWrite(panleft, HIGH);
      }
      count++;

    }
    if (bounce == 5 && stage == 0 && ps2x.ButtonReleased(PSB_L3)) {

      digitalWrite(swingLeft, LOW);
      digitalWrite(panright, LOW);
      digitalWrite(swingRight, LOW);
      digitalWrite(panleft, LOW);

      mocodistance = count;
      count = 0;
      stage = 1;
    }

    if (bounce == 5 && stage == 1) {
      if (count <= mocodistance) {

        //motor OFF
        if (swingSwitch == 0) {
          digitalWrite(swingLeft, LOW);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingRight, LOW);
        }
        if (panswitch == 0) {
          digitalWrite(panright, LOW);
        }
        if (panswitch == 1) {
          digitalWrite(panleft, LOW);
        }
        //motor ON
        if (swingSwitch == 0) {
          digitalWrite(swingRight, HIGH);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingLeft, HIGH);
        }
        if (panswitch == 0) {
          digitalWrite(panleft, HIGH);
        }
        if (panswitch == 1) {
          digitalWrite(panright, HIGH);
        }

        count++;
      }

      if (count >= mocodistance) {

        //motor OFF
        if (swingSwitch == 0) {
          digitalWrite(swingRight, LOW);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingLeft, LOW);
        }
        if (panswitch == 0) {
          digitalWrite(panleft, LOW);
        }
        if (panswitch == 1) {
          digitalWrite(panright, LOW);
        }

        //motor ON
        if (swingSwitch == 0) {
          digitalWrite(swingLeft, HIGH);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingRight, HIGH);
        }
        if (panswitch == 0) {
          digitalWrite(panright, HIGH);
        }
        if (panswitch == 1) {
          digitalWrite(panleft, HIGH);
        }

        count++;
      }

      if (count >= mocodistance * 2) {
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

      if (liftswitch == 0) {
        digitalWrite(liftup, HIGH);
      }
      if (liftswitch == 1) {
        digitalWrite(liftdown, HIGH);
      }

      if (tiltswitch == 0) {
        digitalWrite(tiltdown, HIGH);
      }
      if (tiltswitch == 1) {
        digitalWrite(tiltup, HIGH);
      }
      count++;

    }
    if (bounce == 6 && stage == 0 && ps2x.ButtonReleased(PSB_L3)) {

      digitalWrite(liftup, LOW);
      digitalWrite(tiltdown, LOW);
      digitalWrite(liftdown, LOW);
      digitalWrite(tiltup, LOW);

      mocodistance = count;
      count = 0;
      stage = 1;
    }

    if (bounce == 6 && stage == 1) {
      if (count <= mocodistance) {

        if (liftswitch == 0) {
          digitalWrite(liftup, LOW);
        }
        if (liftswitch == 1) {
          digitalWrite(liftdown, LOW);
        }

        if (tiltswitch == 0) {
          digitalWrite(tiltdown, LOW);
        }
        if (tiltswitch == 1) {
          digitalWrite(tiltup, LOW);
        }


        if (liftswitch == 0) {
          digitalWrite(liftdown, HIGH);
        }
        if (liftswitch == 1) {
          digitalWrite(liftup, HIGH);
        }

        if (tiltswitch == 0) {
          digitalWrite(tiltup, HIGH);
        }
        if (tiltswitch == 1) {
          digitalWrite(tiltdown, HIGH);
        }

        count++;
      }

      if (count >= mocodistance) {

        if (liftswitch == 0) {
          digitalWrite(liftdown, LOW);
        }
        if (liftswitch == 1) {
          digitalWrite(liftup, LOW);
        }

        if (tiltswitch == 0) {
          digitalWrite(tiltup, LOW);
        }
        if (tiltswitch == 1) {
          digitalWrite(tiltdown, LOW);
        }


        if (liftswitch == 0) {
          digitalWrite(liftup, HIGH);
        }
        if (liftswitch == 1) {
          digitalWrite(liftdown, HIGH);
        }

        if (tiltswitch == 0) {
          digitalWrite(tiltdown, HIGH);
        }
        if (tiltswitch == 1) {
          digitalWrite(tiltup, HIGH);
        }
        count++;
      }

      if (count >= mocodistance * 2) {
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
      if (panswitch == 0) {
        digitalWrite(panleft, HIGH);
      }
      if (panswitch == 1) {
        digitalWrite(panright, HIGH);
      }

      count++;
    }

    if (bounce == 7 && stage == 0 && ps2x.ButtonReleased(PSB_L3)) {

      digitalWrite(swingRight, LOW);
      digitalWrite(panleft, LOW);
      digitalWrite(swingLeft, LOW);
      digitalWrite(panright, LOW);


      mocodistance = count;
      count = 0;
      stage = 1;
    }

    if (bounce == 7 && stage == 1) {

      if (count <= mocodistance) {


        if (swingSwitch == 0) {
          digitalWrite(swingRight, LOW);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingLeft, LOW);
        }
        if (panswitch == 0) {
          digitalWrite(panleft, LOW);
        }
        if (panswitch == 1) {
          digitalWrite(panright, LOW);
        }



        if (swingSwitch == 0) {
          digitalWrite(swingLeft, HIGH);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingRight, HIGH);
        }
        if (panswitch == 0) {
          digitalWrite(panright, HIGH);
        }
        if (panswitch == 1) {
          digitalWrite(panleft, HIGH);
        }

        count++;
      }

      if (count >= mocodistance) {

        if (swingSwitch == 0) {
          digitalWrite(swingLeft, LOW);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingRight, LOW);
        }
        if (panswitch == 0) {
          digitalWrite(panright, LOW);
        }
        if (panswitch == 1) {
          digitalWrite(panleft, LOW);
        }


        if (swingSwitch == 0) {
          digitalWrite(swingRight, HIGH);
        }
        if (swingSwitch == 1) {
          digitalWrite(swingLeft, HIGH);
        }
        if (panswitch == 0) {
          digitalWrite(panleft, HIGH);
        }
        if (panswitch == 1) {
          digitalWrite(panright, HIGH);
        }


        count++;
      }
      if (count >= mocodistance * 2) {
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

      if (liftswitch == 0) {
        digitalWrite(liftdown, HIGH);
      }
      if (liftswitch == 1) {
        digitalWrite(liftup, HIGH);
      }

      if (tiltswitch == 0) {
        digitalWrite(tiltup, HIGH);
      }
      if (tiltswitch == 1) {
        digitalWrite(tiltdown, HIGH);
      }

      count++;

    }

    if (bounce == 8 && stage == 0 && ps2x.ButtonReleased(PSB_L3)) {

      digitalWrite(liftup, LOW);
      digitalWrite(tiltdown, LOW);
      digitalWrite(liftdown, LOW);
      digitalWrite(tiltup, LOW);

      mocodistance = count;
      count = 0;
      stage = 1;

    }

    if (bounce == 8 && stage == 1) {
      if (count <= mocodistance) {

        if (liftswitch == 0) {
          digitalWrite(liftdown, LOW);
        }
        if (liftswitch == 1) {
          digitalWrite(liftup, LOW);
        }

        if (tiltswitch == 0) {
          digitalWrite(tiltup, LOW);
        }
        if (tiltswitch == 1) {
          digitalWrite(tiltdown, LOW);
        }

        if (liftswitch == 0) {
          digitalWrite(liftup, HIGH);
        }
        if (liftswitch == 1) {
          digitalWrite(liftdown, HIGH);
        }

        if (tiltswitch == 0) {
          digitalWrite(tiltdown, HIGH);
        }
        if (tiltswitch == 1) {
          digitalWrite(tiltup, HIGH);
        }
        count++;
      }

      if (count >= mocodistance) {

        if (liftswitch == 0) {
          digitalWrite(liftup, LOW);
        }
        if (liftswitch == 1) {
          digitalWrite(liftdown, LOW);
        }

        if (tiltswitch == 0) {
          digitalWrite(tiltdown, LOW);
        }
        if (tiltswitch == 1) {
          digitalWrite(tiltup, LOW);
        }


        if (liftswitch == 0) {
          digitalWrite(liftdown, HIGH);
        }
        if (liftswitch == 1) {
          digitalWrite(liftup, HIGH);
        }

        if (tiltswitch == 0) {
          digitalWrite(tiltup, HIGH);
        }
        if (tiltswitch == 1) {
          digitalWrite(tiltdown, HIGH);
        }
        count++;
      }

      if (count >= mocodistance * 2) {
        count = 0;
      }
    }










  }
  delay(10);
}

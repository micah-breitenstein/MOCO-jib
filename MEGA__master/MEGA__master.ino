#include <PS2X_lib.h>  //for v1.6

#define PS2_DAT 10
#define PS2_CMD 9
#define PS2_SEL 8
#define PS2_CLK 11

//#define pressures true
#define pressures false
//#define rumble true
#define rumble false

PS2X ps2x;
int error = 0;
byte type = 0;
byte vibrate = 0;

///////////////////
//button settings
//////////////////

int swingleft = 24;
int swingright = 25;
int swingspeedup = 26;
int swingspeeddown = 27;
int swingsolo = 0;

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

int swinginmotion = 0;
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

// Consolidated DIP switches
const int DIPSWITCH_PINS[] = {35, 43, 37, 39, 41}; // DIPSWITCH1 to DIPSWITCH5
int DIPSWITCH_STATES[5] = {0};

// Motion switch states
int swingswitch = 0;
int panswitch = 0;
int liftswitch = 0;
int tiltswitch = 0;
int focusswitch = 0;
int delaytime;

void setup() {
  // Serial.begin(57600);

  pinMode(swingleft, OUTPUT);
  pinMode(swingright, OUTPUT);
  pinMode(swingspeedup, OUTPUT);
  pinMode(swingspeeddown, OUTPUT);
  pinMode(liftdown, OUTPUT);
  pinMode(liftup, OUTPUT);
  pinMode(liftspeedup, OUTPUT);
  pinMode(liftspeeddown, OUTPUT);
  pinMode(panleft, OUTPUT);
  pinMode(panright, OUTPUT);
  pinMode(panspeedup, OUTPUT);
  pinMode(panspeeddown, OUTPUT);
  pinMode(tiltdown, OUTPUT);
  pinMode(tiltup, OUTPUT);
  pinMode(tiltspeedup, OUTPUT);
  pinMode(tiltspeeddown, OUTPUT);
  pinMode(focusleft, OUTPUT);
  pinMode(focusright, OUTPUT);
  pinMode(focusspeedup, OUTPUT);
  pinMode(focusspeeddown, OUTPUT);

  pinMode(trigger, OUTPUT);
  interval = intervalseconds * 1000;

  // Configure all DIP switches as INPUT_PULLUP
  for (int i = 0; i < 5; i++) {
    pinMode(DIPSWITCH_PINS[i], INPUT_PULLUP);
  }

  pinMode(tiltspeeduponly, OUTPUT);
  pinMode(tiltspeeddownonly, OUTPUT);
  pinMode(panspeeduponly, OUTPUT);
  pinMode(panspeeddownonly, OUTPUT);

  delay(300);

  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);

  if (error == 0) {
    Serial.print("Found Controller, configured successful ");
    Serial.print("pressures = ");
    if (pressures) Serial.println("true ");
    else Serial.println("false");
    Serial.print("rumble = ");
    if (rumble) Serial.println("true);");
    else Serial.println("false");
    Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
    Serial.println("holding L1 or R1 will print out the analog stick values.");
    Serial.println("Note: Go to www.billporter.info for updates and to report bugs.");
  } else if (error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
  else if (error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
  else if (error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");

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
  if (error == 1) return; // Skip loop if no controller found

  if (type != 2) {  // DualShock Controller
    ps2x.read_gamepad(false, vibrate); // Unnecessary vibration

    // Read DIP switch states
    for (int i = 0; i < 5; i++) {
      DIPSWITCH_STATES[i] = digitalRead(DIPSWITCH_PINS[i]);
    }

    // Populate Motion switches based on DIP switches
    swingswitch = DIPSWITCH_STATES[0];
    panswitch = DIPSWITCH_STATES[1];
    liftswitch = DIPSWITCH_STATES[2];
    tiltswitch = DIPSWITCH_STATES[3];
    focusswitch = DIPSWITCH_STATES[4];

    // Add the rest of your loop logic here...
  }

  delay(10);
}
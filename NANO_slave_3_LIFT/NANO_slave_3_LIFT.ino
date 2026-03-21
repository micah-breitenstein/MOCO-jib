// NANO Slave 3 — Boom Lift axis
// Receives direction and speed commands from the Mega via digital pins.
// Controls a stepper motor driver (DIR + PUL) for boom lift movement.

///// PIN ASSIGNMENTS
const int driverDIR    = 2;  // stepper driver direction pin
const int driverPUL    = 4;  // stepper driver pulse pin
const int upButton     = 7;  // HIGH = lift up command from Mega
const int downButton   = 6;  // HIGH = lift down command from Mega
const int speedUpPin   = 8;  // HIGH = increase speed stage
const int speedDownPin = 9;  // HIGH = decrease speed stage

///// SPEED STAGE SETTINGS
// Step pulse delay in microseconds per stage (0 = slowest, 4 = fastest).
// Lower value = shorter delay between pulses = faster motor.
const int STAGE_COUNT = 5;
const int STAGE_DELAYS[STAGE_COUNT] = {5000, 2500, 1000, 500, 250};

///// STATE
int stage = 0;
int count = STAGE_DELAYS[0];
int pd;
int lastSpeedUp   = 0;
int lastSpeedDown = 0;

///// HELPERS

// Send one step pulse to the stepper driver in the given direction.
// dirHigh=true moves one way; dirHigh=false moves the other.
void stepMotor(bool dirHigh) {
  digitalWrite(driverDIR, dirHigh ? HIGH : LOW);
  digitalWrite(driverPUL, LOW);
  digitalWrite(LED_BUILTIN, HIGH);
  delayMicroseconds(pd);
  digitalWrite(driverPUL, HIGH);
  delayMicroseconds(pd);
}

// Handle speed stage transitions.
// Note: stage 0 → 1 fires continuously while held; all other transitions
// require a rising edge to prevent multiple increments per press.
void updateSpeedStage(int speedUpRead, int speedDownRead) {
  if (lastSpeedDown == 0 && speedDownRead == 1 && stage > 0) {
    stage--;
    count = STAGE_DELAYS[stage];
    Serial.print("STAGE"); Serial.println(stage);
  }

  if (stage == 0 && speedUpRead == 1) {
    stage = 1;
    count = STAGE_DELAYS[1];
    Serial.println("STAGE1");
  } else if (lastSpeedUp == 0 && speedUpRead == 1 && stage < STAGE_COUNT - 1) {
    stage++;
    count = STAGE_DELAYS[stage];
    Serial.print("STAGE"); Serial.println(stage);
  }

  lastSpeedUp   = speedUpRead;
  lastSpeedDown = speedDownRead;
}

void setup() {
  // Serial.begin(57600);
  pinMode(driverDIR,    OUTPUT);
  pinMode(driverPUL,    OUTPUT);
  pinMode(speedUpPin,   INPUT);
  pinMode(speedDownPin, INPUT);
  pinMode(upButton,     INPUT);
  pinMode(downButton,   INPUT);
  pinMode(LED_BUILTIN,  OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, LOW);

  int upRead    = digitalRead(upButton);
  int downRead  = digitalRead(downButton);
  int speedUp   = digitalRead(speedUpPin);
  int speedDown = digitalRead(speedDownPin);

  ///// SPEED STAGE CHANGES
  updateSpeedStage(speedUp, speedDown);

  pd = count;

  ///// MOTOR MOVEMENTS
  if (upRead == HIGH) {
    Serial.println("LIFT UP");
    stepMotor(true);
  }

  if (downRead == HIGH) {
    Serial.println("LIFT DOWN");
    stepMotor(false);
  }
}

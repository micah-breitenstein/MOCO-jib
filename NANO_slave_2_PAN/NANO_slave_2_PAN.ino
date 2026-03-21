// NANO Slave 2 — Camera Pan axis
// Receives direction and speed commands from the Mega via digital pins.
// Supports coarse speed stages, fine-tune adjustment, and a high-speed bypass mode.

///// PIN ASSIGNMENTS
const int driverDIR    = 2;   // stepper driver direction pin
const int driverPUL    = 4;   // stepper driver pulse pin
const int upButton     = 7;   // HIGH = pan left command from Mega
const int downButton   = 6;   // HIGH = pan right command from Mega
const int speedUpPin   = 8;   // HIGH = increase speed stage
const int speedDownPin = 9;   // HIGH = decrease speed stage
const int speedAdjUp   = 11;  // fine-tune: decrease pulse delay (faster)
const int speedAdjDown = 12;  // fine-tune: increase pulse delay (slower)

///// SPEED STAGE SETTINGS
// Step pulse delay in microseconds per stage (0 = slowest, 4 = fastest).
// Lower value = shorter delay between pulses = faster motor.
const int STAGE_COUNT     = 5;
const int STAGE_DELAYS[STAGE_COUNT] = {3000, 1700, 650, 300, 150};
const int HIGH_SPEED_DELAY = 100;  // used when both adj pins are HIGH (solo axis trim)
const int MIN_DELAY        = 100;  // floor for fine adjustment

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

// Handle fine speed adjustment and high-speed bypass mode.
// Both adj pins HIGH → override pd with HIGH_SPEED_DELAY (used for solo axis trim).
// Only adjUp HIGH → decrease delay (faster). Only adjDown HIGH → increase delay (slower).
// Returns the pulse delay to use this tick.
int applySpeedAdjust(int adjUpRead, int adjDownRead) {
  if (adjUpRead == 1 && adjDownRead == 1) {
    Serial.println("HIGH SPEED MODE");
    return HIGH_SPEED_DELAY;
  }
  if (adjUpRead == 1 && adjDownRead == 0) {
    Serial.println("SPEED UP ADJUST");
    count--;
    if (count < MIN_DELAY) count = MIN_DELAY;
  }
  if (adjDownRead == 1 && adjUpRead == 0) {
    Serial.println("SPEED DOWN ADJUST");
    count++;
  }
  return count;
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
  pinMode(speedAdjUp,   INPUT);
  pinMode(speedAdjDown, INPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, LOW);

  int upRead    = digitalRead(upButton);
  int downRead  = digitalRead(downButton);
  int speedUp   = digitalRead(speedUpPin);
  int speedDown = digitalRead(speedDownPin);
  int adjUp     = digitalRead(speedAdjUp);
  int adjDown   = digitalRead(speedAdjDown);

  ///// SPEED STAGE CHANGES
  updateSpeedStage(speedUp, speedDown);

  ///// SPEED ADJUSTMENTS + HIGH SPEED MODE
  pd = applySpeedAdjust(adjUp, adjDown);

  ///// MOTOR MOVEMENTS
  if (upRead == HIGH) {
    Serial.println("PAN LEFT");
    stepMotor(true);
  }

  if (downRead == HIGH) {
    Serial.println("PAN RIGHT");
    stepMotor(false);
  }
}

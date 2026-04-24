// NANO Slave 5 — Camera Focus axis
// Receives direction and speed commands from the Mega via digital pins.
// Controls a stepper motor driver (DIR + PUL) for focus ring movement.
// Features: non-blocking motion, acceleration/deceleration ramping.

constexpr bool FOCUS_SERIAL_DEBUG = false;

///// PIN ASSIGNMENTS
const int driverDIR    = 2;  // stepper driver direction pin
const int driverPUL    = 4;  // stepper driver pulse pin
const int upButton     = 7;  // HIGH = focus one direction from Mega
const int downButton   = 6;  // HIGH = focus other direction from Mega
const int speedUpPin   = 8;  // HIGH = increase speed stage
const int speedDownPin = 9;  // HIGH = decrease speed stage

///// SPEED STAGE SETTINGS
// Step pulse delay in microseconds per stage (0 = slowest, 2 = fastest).
// Lower value = shorter delay between pulses = faster motor.
const int STAGE_COUNT = 3;
const int STAGE_DELAYS[STAGE_COUNT] = {2500, 800, 100};

///// RAMPING SETTINGS
constexpr int RAMP_INCREMENT = 120;
constexpr int RAMP_START_DELAY = 4000;

///// STATE
int stage = 0;
int lastSpeedUp   = 0;
int lastSpeedDown = 0;
bool lastDirectionConflict = false;
unsigned long speedIndicatorUntilMs = 0;

// Motion ramping state
int targetDelay = STAGE_DELAYS[0];
int currentDelay = STAGE_DELAYS[0];
bool motionActive = false;
bool rampingDown = false;
unsigned long lastStepMicros = 0;
unsigned long rampUpdateMicros = 0;

void debugLog(const char* message) {
  if (FOCUS_SERIAL_DEBUG) {
    Serial.println(message);
  }
}

void debugLogStage(int stageValue) {
  if (FOCUS_SERIAL_DEBUG) {
    Serial.print("STAGE");
    Serial.println(stageValue);
  }
}

void triggerSpeedIndicatorPulse() {
  speedIndicatorUntilMs = millis() + 120;
}

void debugLogDelay(int delayValue) {
  if (FOCUS_SERIAL_DEBUG) {
    Serial.print("DELAY");
    Serial.println(delayValue);
  }
}

///// HELPERS

bool stepMotorNonBlocking(bool dirHigh, unsigned long nowMicros) {
  if (nowMicros - lastStepMicros >= currentDelay) {
    digitalWrite(driverDIR, dirHigh ? HIGH : LOW);
    digitalWrite(driverPUL, LOW);
    delayMicroseconds(1);
    digitalWrite(driverPUL, HIGH);
    lastStepMicros = nowMicros;
    return true;
  }
  return false;
}

void updateRamping(unsigned long nowMicros) {
  if (nowMicros - rampUpdateMicros < 1000) {
    return;
  }
  rampUpdateMicros = nowMicros;

  if (currentDelay < targetDelay) {
    currentDelay = min(currentDelay + RAMP_INCREMENT, targetDelay);
  } else if (currentDelay > targetDelay) {
    currentDelay = max(currentDelay - RAMP_INCREMENT, targetDelay);
  }
}

void startMotion() {
  if (!motionActive) {
    motionActive = true;
    currentDelay = RAMP_START_DELAY;
    rampingDown = false;
    lastStepMicros = 0;
    rampUpdateMicros = 0;
    digitalWrite(LED_BUILTIN, HIGH);
    debugLog("MOTION START");
  }
}

void stopMotion() {
  if (motionActive && !rampingDown) {
    rampingDown = true;
    targetDelay = RAMP_START_DELAY;
    debugLog("MOTION STOP");
  }
}

// Handle speed stage transitions. All transitions require a rising edge.
void updateSpeedStage(int speedUpRead, int speedDownRead) {
  if (lastSpeedDown == 0 && speedDownRead == 1 && stage > 0) {
    stage--;
    targetDelay = STAGE_DELAYS[stage];
    debugLogStage(stage);
    debugLogDelay(targetDelay);
    triggerSpeedIndicatorPulse();
  }

  if (lastSpeedUp == 0 && speedUpRead == 1 && stage < STAGE_COUNT - 1) {
    stage++;
    targetDelay = STAGE_DELAYS[stage];
    debugLogStage(stage);
    debugLogDelay(targetDelay);
    triggerSpeedIndicatorPulse();
  }

  lastSpeedUp   = speedUpRead;
  lastSpeedDown = speedDownRead;
}

void setup() {
  if (FOCUS_SERIAL_DEBUG) {
    Serial.begin(57600);
  }
  pinMode(driverDIR,    OUTPUT);
  pinMode(driverPUL,    OUTPUT);
  pinMode(speedUpPin,   INPUT);
  pinMode(speedDownPin, INPUT);
  pinMode(upButton,     INPUT);
  pinMode(downButton,   INPUT);
  pinMode(LED_BUILTIN,  OUTPUT);
}

void loop() {
  if (millis() < speedIndicatorUntilMs) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else if (!motionActive) {
    digitalWrite(LED_BUILTIN, LOW);
  }

  unsigned long nowMicros = micros();

  int upRead    = digitalRead(upButton);
  int downRead  = digitalRead(downButton);
  int speedUp   = digitalRead(speedUpPin);
  int speedDown = digitalRead(speedDownPin);

  ///// SPEED STAGE CHANGES
  updateSpeedStage(speedUp, speedDown);

  ///// DIRECTION CONFLICT CHECK
  bool directionConflict = (upRead == HIGH && downRead == HIGH);
  if (directionConflict) {
    if (!lastDirectionConflict) {
      debugLog("FOCUS CONFLICT: in + out command active; movement skipped");
    }
    lastDirectionConflict = true;
    stopMotion();
    return;
  }
  lastDirectionConflict = false;

  ///// MOTION LOGIC
  bool commandActive = (upRead == HIGH || downRead == HIGH);
  
  if (!commandActive) {
    stopMotion();
  } else if (!motionActive) {
    startMotion();
  }

  if (motionActive) {
    updateRamping(nowMicros);

    if (rampingDown && currentDelay >= RAMP_START_DELAY) {
      motionActive = false;
      rampingDown = false;
      digitalWrite(LED_BUILTIN, LOW);
      debugLog("MOTION STOPPED");
      return;
    }

    if (upRead == HIGH) {
      stepMotorNonBlocking(true, nowMicros);
    } else if (downRead == HIGH) {
      stepMotorNonBlocking(false, nowMicros);
    }
  }
}

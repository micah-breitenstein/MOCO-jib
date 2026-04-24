// NANO Slave 1 — Boom Swing axis
// Receives direction and speed commands from the Mega via digital pins.
// Controls a stepper motor driver (DIR + PUL) for boom swing movement.
// Features: non-blocking motion, acceleration/deceleration ramping.

constexpr bool SWING_SERIAL_DEBUG = false;

///// PIN ASSIGNMENTS
const int driverDIR    = 2;  // stepper driver direction pin
const int driverPUL    = 4;  // stepper driver pulse pin
const int upButton     = 7;  // HIGH = swing left command from Mega
const int downButton   = 6;  // HIGH = swing right command from Mega
const int speedUpPin   = 8;  // HIGH = increase speed stage
const int speedDownPin = 9;  // HIGH = decrease speed stage

///// SPEED STAGE SETTINGS
// Step pulse delay in microseconds per stage (0 = slowest, 4 = fastest).
// Lower value = shorter delay between pulses = faster motor.
const int STAGE_COUNT = 5;
const int STAGE_DELAYS[STAGE_COUNT] = {5000, 2500, 1000, 500, 250};

///// RAMPING SETTINGS
// Ramp acceleration: microseconds per update step
// Higher = slower ramp, lower = faster ramp to target speed
constexpr int RAMP_INCREMENT = 150;  // reduce delay by this amount per ramp step
constexpr int RAMP_START_DELAY = 8000;  // start acceleration from this delay

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
  if (SWING_SERIAL_DEBUG) {
    Serial.println(message);
  }
}

void debugLogStage(int stageValue) {
  if (SWING_SERIAL_DEBUG) {
    Serial.print("STAGE");
    Serial.println(stageValue);
  }
}

void triggerSpeedIndicatorPulse() {
  speedIndicatorUntilMs = millis() + 120;
}

void debugLogDelay(int delayValue) {
  if (SWING_SERIAL_DEBUG) {
    Serial.print("DELAY");
    Serial.println(delayValue);
  }
}

///// HELPERS

// Non-blocking stepper pulse using micros() timing.
// Returns true if a pulse was sent, false if still in delay period.
bool stepMotorNonBlocking(bool dirHigh, unsigned long nowMicros) {
  if (nowMicros - lastStepMicros >= currentDelay) {
    digitalWrite(driverDIR, dirHigh ? HIGH : LOW);
    digitalWrite(driverPUL, LOW);
    delayMicroseconds(1);  // very brief pulse
    digitalWrite(driverPUL, HIGH);
    lastStepMicros = nowMicros;
    return true;
  }
  return false;
}

// Update ramping progression: gradually move currentDelay toward targetDelay.
void updateRamping(unsigned long nowMicros) {
  if (nowMicros - rampUpdateMicros < 1000) {
    return;  // ramp updates every ~1ms
  }
  rampUpdateMicros = nowMicros;

  if (currentDelay < targetDelay) {
    // Ramping up (slowing down)
    currentDelay = min(currentDelay + RAMP_INCREMENT, targetDelay);
  } else if (currentDelay > targetDelay) {
    // Ramping down (speeding up)
    currentDelay = max(currentDelay - RAMP_INCREMENT, targetDelay);
  }
}

// Begin motion with acceleration from a slower start speed.
void startMotion() {
  if (!motionActive) {
    motionActive = true;
    currentDelay = RAMP_START_DELAY;  // start from slow ramp speed
    rampingDown = false;
    lastStepMicros = 0;
    rampUpdateMicros = 0;
    digitalWrite(LED_BUILTIN, HIGH);
    debugLog("MOTION START");
  }
}

// Trigger deceleration and eventual stop.
void stopMotion() {
  if (motionActive && !rampingDown) {
    rampingDown = true;
    targetDelay = RAMP_START_DELAY;  // ramp back to slow speed
    debugLog("MOTION STOP");
  }
}

// Handle speed stage transitions.
// Note: stage 0 → 1 fires continuously while held; all other transitions
// require a rising edge to prevent multiple increments per press.
void updateSpeedStage(int speedUpRead, int speedDownRead) {
  if (lastSpeedDown == 0 && speedDownRead == 1 && stage > 0) {
    stage--;
    targetDelay = STAGE_DELAYS[stage];
    debugLogStage(stage);
    debugLogDelay(targetDelay);
    triggerSpeedIndicatorPulse();
  }

  if (stage == 0 && speedUpRead == 1) {
    stage = 1;
    targetDelay = STAGE_DELAYS[1];
    debugLog("STAGE1");
    debugLogDelay(targetDelay);
    triggerSpeedIndicatorPulse();
  } else if (lastSpeedUp == 0 && speedUpRead == 1 && stage < STAGE_COUNT - 1) {
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
  if (SWING_SERIAL_DEBUG) {
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
  // Speed indicator LED for stage changes
  if (millis() < speedIndicatorUntilMs) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else if (!motionActive) {
    digitalWrite(LED_BUILTIN, LOW);
  }
  // LED stays HIGH while motionActive (set in startMotion/updateRamping)

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
      debugLog("SWING CONFLICT: left + right command active; movement skipped");
    }
    lastDirectionConflict = true;
    stopMotion();
    return;
  }
  lastDirectionConflict = false;

  ///// MOTION LOGIC
  bool commandActive = (upRead == HIGH || downRead == HIGH);
  
  if (!commandActive) {
    // No motion command
    stopMotion();
  } else if (!motionActive) {
    // Command is active and motion not yet started
    startMotion();
  }

  // Update ramping if motion is active
  if (motionActive) {
    updateRamping(nowMicros);

    // Finish deceleration and stop
    if (rampingDown && currentDelay >= RAMP_START_DELAY) {
      motionActive = false;
      rampingDown = false;
      digitalWrite(LED_BUILTIN, LOW);
      debugLog("MOTION STOPPED");
      return;
    }

    // Send step pulse if enough time has elapsed
    if (upRead == HIGH) {
      stepMotorNonBlocking(true, nowMicros);  // true = left
    } else if (downRead == HIGH) {
      stepMotorNonBlocking(false, nowMicros);  // false = right
    }
  }
}

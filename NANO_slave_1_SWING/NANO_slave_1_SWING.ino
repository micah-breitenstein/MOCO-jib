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
constexpr int RAMP_STOP_DELAY  = 8000;  // decelerate toward this delay before stop

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
bool pendingStop = false;
unsigned long lastStepMicros = 0;
unsigned long rampUpdateMicros = 0;

// Direction state: avoid instant reverse while moving.
bool currentDirection = true;
bool requestedDirection = true;
bool directionChangePending = false;

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
    delayMicroseconds(2);  // safer pulse width for common stepper drivers
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
    pendingStop = false;
    lastStepMicros = 0;
    rampUpdateMicros = 0;
    targetDelay = STAGE_DELAYS[stage];
    debugLog("MOTION START");
  }
}

// Trigger deceleration and eventual stop.
void stopMotion() {
  if (motionActive && !pendingStop) {
    pendingStop = true;
    targetDelay = RAMP_STOP_DELAY;  // ramp back to slow speed before stopping
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
  // Unified LED policy: on while moving or during speed-change pulse.
  bool ledOn = motionActive || (millis() < speedIndicatorUntilMs);
  digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);

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
    // No motion command -> clean ramp-down stop.
    stopMotion();
  } else {
    requestedDirection = (upRead == HIGH);

    if (!motionActive) {
      // Start movement in requested direction.
      currentDirection = requestedDirection;
      startMotion();
    } else if (!pendingStop && requestedDirection != currentDirection) {
      // Direction change flow:
      // 1) decelerate to stop, 2) flip direction, 3) ramp up again.
      directionChangePending = true;
      stopMotion();
    }
  }

  // Update ramping if motion is active
  if (motionActive) {
    updateRamping(nowMicros);

    // Finish deceleration and stop
    if (pendingStop && currentDelay >= RAMP_STOP_DELAY) {
      motionActive = false;
      pendingStop = false;
      debugLog("MOTION STOPPED");

      // After a safe stop, apply deferred direction change and restart smoothly.
      if (directionChangePending) {
        currentDirection = requestedDirection;
        directionChangePending = false;
        startMotion();
      }
      return;
    }

    // Send step pulse if enough time has elapsed
    if (!pendingStop && commandActive) {
      stepMotorNonBlocking(currentDirection, nowMicros);
    }
  }
}

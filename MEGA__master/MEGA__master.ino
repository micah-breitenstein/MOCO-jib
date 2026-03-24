#include <PS2X_lib.h>  //for v1.6

constexpr uint8_t PS2_DAT = 10;
constexpr uint8_t PS2_CMD = 9;
constexpr uint8_t PS2_SEL = 8;
constexpr uint8_t PS2_CLK = 11;

constexpr bool PRESSURES = false;
constexpr bool RUMBLE = true;
constexpr bool DEBUG_EDGE_EVENTS = false;

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

bool swingInMotion = false;
bool liftInMotion = false;

int leftStickXvalue;
int leftStickYvalue;
int rightStickXvalue;
int rightStickYvalue;

// Timelapse Variables
int timelapseMode = 0;
int timelapseIntervalSeconds = 15;
unsigned long timelapseIntervalMs;
int stepDist = 100;
const uint8_t trigger = 28;

enum TimelapsePhase {
  TIMELAPSE_PHASE_IDLE,
  TIMELAPSE_PHASE_TRIGGER_LOW,
  TIMELAPSE_PHASE_TRIGGER_HIGH,
  TIMELAPSE_PHASE_MOVE_ACTIVE
};

TimelapsePhase timelapsePhase = TIMELAPSE_PHASE_IDLE;
unsigned long timelapsePhaseStartMs = 0;

enum IntervalRumblePhase {
  INTERVAL_RUMBLE_IDLE,
  INTERVAL_RUMBLE_LONG_ACTIVE,
  INTERVAL_RUMBLE_SHORT_ACTIVE,
  INTERVAL_RUMBLE_SHORT_PAUSE
};

enum StepDistRumblePhase {
  STEP_DIST_RUMBLE_IDLE,
  STEP_DIST_RUMBLE_LONG_ACTIVE,
  STEP_DIST_RUMBLE_LONG_PAUSE
};

enum PendingRumblePattern {
  PENDING_RUMBLE_NONE,
  PENDING_RUMBLE_INTERVAL,
  PENDING_RUMBLE_STEP_DIST
};

enum FeedbackRumblePhase {
  FEEDBACK_RUMBLE_IDLE,
  FEEDBACK_RUMBLE_ON,
  FEEDBACK_RUMBLE_PAUSE
};

IntervalRumblePhase intervalRumblePhase = INTERVAL_RUMBLE_IDLE;
unsigned long intervalRumblePhaseStartMs = 0;
uint8_t intervalRumbleLongsRemaining = 0;
uint8_t intervalRumbleShortsRemaining = 0;
StepDistRumblePhase stepDistRumblePhase = STEP_DIST_RUMBLE_IDLE;
unsigned long stepDistRumblePhaseStartMs = 0;
uint8_t stepDistRumbleLongsRemaining = 0;
PendingRumblePattern pendingRumblePattern = PENDING_RUMBLE_NONE;
bool rumbleSeparatorActive = false;
unsigned long rumbleSeparatorStartMs = 0;
FeedbackRumblePhase feedbackRumblePhase = FEEDBACK_RUMBLE_IDLE;
unsigned long feedbackRumblePhaseStartMs = 0;
uint8_t feedbackRumblePulsesRemaining = 0;
unsigned long feedbackRumbleOnMs = 0;
unsigned long feedbackRumbleTotalMs = 0;

// Motion Control Variables
int bounce = 0;
int stage = 0;
unsigned long bouncePhaseStartMs = 0;
unsigned long bounceMoveDurationMs = 0;

constexpr uint8_t DIP_SWITCH_1 = 35;
constexpr uint8_t DIP_SWITCH_2 = 43;
constexpr uint8_t DIP_SWITCH_3 = 37;
constexpr uint8_t DIP_SWITCH_4 = 39;
constexpr uint8_t DIP_SWITCH_5 = 41;
constexpr unsigned long CONTROLLER_STARTUP_DELAY_MS = 300;
constexpr unsigned long CONTROLLER_RETRY_INTERVAL_MS = 2000;
constexpr int STICK_MIN = 0;
constexpr int STICK_CENTER = 128;
constexpr int STICK_MAX = 255;
constexpr int STICK_MODE_LOW_THRESHOLD = 123;
constexpr int STICK_MODE_HIGH_THRESHOLD = 133;
constexpr int PAN_STOP_NONE = 0;
constexpr int PAN_STOP_ACTIVE = 1;
constexpr int PAN_STOP_TRIM = 2;
constexpr int TILT_STOP_NONE = 0;
constexpr int TILT_STOP_ACTIVE = 1;
constexpr int TILT_STOP_TRIM = 2;
constexpr int TIMELAPSE_INTERVAL_MIN_SECONDS = 1;
constexpr int TIMELAPSE_INTERVAL_MAX_SECONDS = 99;
constexpr int TIMELAPSE_STEP_DIST_MIN_MS = 20;
constexpr int TIMELAPSE_STEP_DIST_MAX_MS = 150;
constexpr int TIMELAPSE_STEP_DIST_ADJUST_INCREMENT_MS = 10;
constexpr uint8_t RUMBLE_ACTIVE_INTENSITY = 255;
constexpr unsigned long INTERVAL_RUMBLE_LONG_MS = 10000;
constexpr unsigned long INTERVAL_RUMBLE_SHORT_ON_MS = 200;
constexpr unsigned long INTERVAL_RUMBLE_SHORT_TOTAL_MS = 1000;
constexpr unsigned long STEP_DIST_RUMBLE_LONG_ON_MS = 600;
constexpr unsigned long STEP_DIST_RUMBLE_LONG_TOTAL_MS = 1000;
constexpr unsigned long RUMBLE_PATTERN_SEPARATOR_MS = 300;
constexpr unsigned long FEEDBACK_RUMBLE_ON_MS = 120;
constexpr unsigned long FEEDBACK_RUMBLE_TOTAL_MS = 240;
constexpr uint8_t LIMIT_REACHED_RUMBLE_PULSES = 2;
constexpr uint8_t LOCKOUT_DENIED_RUMBLE_PULSES = 3;
constexpr unsigned long EMERGENCY_RELEASE_RUMBLE_ON_MS = 100;
constexpr unsigned long EMERGENCY_RELEASE_RUMBLE_TOTAL_MS = 180;
constexpr uint8_t EMERGENCY_RELEASE_RUMBLE_PULSES = 1;
constexpr unsigned long L3_ENDPOINT_RUMBLE_ON_MS = 200;
constexpr unsigned long L3_ENDPOINT_RUMBLE_TOTAL_MS = 350;
constexpr uint8_t L3_ENDPOINT_RUMBLE_PULSES = 2;
constexpr unsigned long BOUNCE_MIN_MOVE_DURATION_MS = 150;

// Drone mode constants
constexpr unsigned long DRONE_MODE_ENTER_RUMBLE_ON_MS = 200;
constexpr unsigned long DRONE_MODE_ENTER_RUMBLE_TOTAL_MS = 350;
constexpr uint8_t DRONE_MODE_ENTER_RUMBLE_PULSES = 1;
constexpr unsigned long DRONE_MODE_EXIT_RUMBLE_ON_MS = 150;
constexpr unsigned long DRONE_MODE_EXIT_RUMBLE_TOTAL_MS = 300;
constexpr uint8_t DRONE_MODE_EXIT_RUMBLE_PULSES = 2;
constexpr uint8_t DRONE_SPEED_TIER_STOP = 0;
constexpr uint8_t DRONE_SPEED_TIER_MED = 1;
constexpr uint8_t DRONE_SPEED_TIER_HIGH = 2;
constexpr int DRONE_SPEED_TIER_MED_THRESHOLD = 43;
constexpr int DRONE_SPEED_TIER_HIGH_THRESHOLD = 86;
constexpr int DRONE_STICK_MAX_DEFLECTION = 127;
constexpr uint8_t DRONE_EXPO_PERCENT = 65;
constexpr uint8_t DRONE_SWING_EXPO_PERCENT = DRONE_EXPO_PERCENT;
constexpr uint8_t DRONE_LIFT_EXPO_PERCENT = DRONE_EXPO_PERCENT;
constexpr uint8_t DRONE_PAN_EXPO_PERCENT = DRONE_EXPO_PERCENT;
constexpr uint8_t DRONE_TILT_EXPO_PERCENT = DRONE_EXPO_PERCENT;
constexpr int DRONE_SWING_DEADBAND = 12;
constexpr int DRONE_LIFT_DEADBAND = 14;
constexpr int DRONE_PAN_DEADBAND = 10;
constexpr int DRONE_TILT_DEADBAND = 10;
constexpr uint8_t DRONE_SWING_MAX_SPEED_TIER = DRONE_SPEED_TIER_MED;
constexpr uint8_t DRONE_LIFT_MAX_SPEED_TIER = DRONE_SPEED_TIER_MED;
constexpr uint8_t DRONE_PAN_MAX_SPEED_TIER = DRONE_SPEED_TIER_HIGH;
constexpr uint8_t DRONE_TILT_MAX_SPEED_TIER = DRONE_SPEED_TIER_MED;
constexpr bool DRONE_ENABLE_PRECISION_MODIFIER = true;
constexpr bool DRONE_ENABLE_BOOST_MODIFIER = true;
constexpr bool DRONE_L2_R2_NEUTRAL_MODE = true;
constexpr unsigned long DRONE_IDLE_TIMEOUT_MS = 30000UL; // 0 = disabled
constexpr bool DRONE_SERIAL_LOG_ENABLED = true; // set false to silence runtime drone logs

constexpr uint8_t FLOWLAPSE_MAX_WAYPOINTS = 8;
constexpr unsigned long FLOWLAPSE_WAYPOINT_RUMBLE_ON_MS = 90;
constexpr unsigned long FLOWLAPSE_WAYPOINT_RUMBLE_TOTAL_MS = 180;
constexpr uint8_t FLOWLAPSE_WAYPOINT_RUMBLE_PULSES = 1;
constexpr unsigned long FLOWLAPSE_PREVIEW_POINT_HOLD_MS = 700;
constexpr uint8_t FLOWLAPSE_MAX_SPEED_TIER = DRONE_SPEED_TIER_MED;
constexpr unsigned long FLOWLAPSE_TIER_RAMP_INTERVAL_MS = 450;
constexpr float FLOWLAPSE_AXIS_STOP_TOLERANCE = 1.0f;
constexpr float FLOWLAPSE_AXIS_MED_ERROR = 4.0f;
constexpr float FLOWLAPSE_AXIS_HIGH_ERROR = 12.0f;
constexpr float FLOWLAPSE_MIN_WAYPOINT_SEPARATION = 2.5f;
constexpr float FLOWLAPSE_MANUAL_TRACK_RATE_UNITS_PER_SEC = 14.0f;
constexpr float FLOWLAPSE_MED_RATE_UNITS_PER_SEC = 10.0f;
constexpr float FLOWLAPSE_HIGH_RATE_UNITS_PER_SEC = 18.0f;
constexpr unsigned long FLOWLAPSE_CAPTURE_PROGRESS_LOG_MS = 2000;

struct FlowlapseWaypoint {
  float swing;
  float lift;
  float pan;
  float tilt;
};

enum FlowlapseState {
  FLOWLAPSE_STATE_RECORDING,
  FLOWLAPSE_STATE_READY_FOR_PREVIEW,
  FLOWLAPSE_STATE_PREVIEW_RUNNING,
  FLOWLAPSE_STATE_READY_FOR_CAPTURE,
  FLOWLAPSE_STATE_CAPTURE_RUNNING
};

enum FlowlapseCapturePhase {
  FLOWLAPSE_CAPTURE_TRIGGER_LOW,
  FLOWLAPSE_CAPTURE_TRIGGER_HIGH,
  FLOWLAPSE_CAPTURE_MOVE_ACTIVE
};

bool isSwingReversed = false;
bool isPanReversed = false;
bool isLiftReversed = false;
bool isTiltReversed = false;
bool isFocusReversed = false;
bool droneMode = false;
bool lastIntervalAdjustUpComboActive = false;
bool lastIntervalAdjustDownComboActive = false;
bool lastStepDistAdjustUpComboActive = false;
bool lastStepDistAdjustDownComboActive = false;
bool lastEmergencyStopComboActive = false;
bool lastRumbleMuteToggleComboActive = false;
bool rumbleMuted = false;
bool suppressNextSelectRelease = false;
bool suppressNextStartRelease = false;
bool lastSettingsReplayComboActive = false;
bool lastDronePrecisionModeActive = false;
bool lastDroneBoostModeActive = false;
bool lastDroneSwingActive = false;
bool lastDroneLiftActive = false;
bool lastDronePanActive = false;
bool lastDroneTiltActive = false;
bool lastFlowlapseClearComboActive = false;
unsigned long droneLastActivityMs = 0;
FlowlapseWaypoint flowlapseWaypoints[FLOWLAPSE_MAX_WAYPOINTS];
uint8_t flowlapseWaypointCount = 0;
FlowlapseState flowlapseState = FLOWLAPSE_STATE_RECORDING;
FlowlapseCapturePhase flowlapseCapturePhase = FLOWLAPSE_CAPTURE_TRIGGER_LOW;
unsigned long flowlapseCapturePhaseStartMs = 0;
unsigned long flowlapsePreviewHoldUntilMs = 0;
unsigned long flowlapseLastUpdateMs = 0;
unsigned long flowlapseLastProgressLogMs = 0;
uint8_t flowlapseTargetWaypointIndex = 0;
bool flowlapseCaptureAlignedToFirstWaypoint = false;

float flowlapseCurrentSwingPos = 0.0f;
float flowlapseCurrentLiftPos = 0.0f;
float flowlapseCurrentPanPos = 0.0f;
float flowlapseCurrentTiltPos = 0.0f;

uint8_t flowlapseSwingTier = DRONE_SPEED_TIER_STOP;
uint8_t flowlapseLiftTier = DRONE_SPEED_TIER_STOP;
uint8_t flowlapsePanTier = DRONE_SPEED_TIER_STOP;
uint8_t flowlapseTiltTier = DRONE_SPEED_TIER_STOP;
unsigned long flowlapseSwingTierLastChangeMs = 0;
unsigned long flowlapseLiftTierLastChangeMs = 0;
unsigned long flowlapsePanTierLastChangeMs = 0;
unsigned long flowlapseTiltTierLastChangeMs = 0;

bool chainStepDistAfterInterval = false;
unsigned long lastControllerRetryMs = 0;
bool unsupportedControllerWarningShown = false;

void configureController() {

  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, PRESSURES, RUMBLE);

  if (error == 0) {
    Serial.print("Controller configured. pressures=");
    Serial.print(PRESSURES ? "true" : "false");
    Serial.print(", rumble=");
    Serial.println(RUMBLE ? "true" : "false");
  }
  else if (error == 1)
    Serial.println("No controller found, check wiring.");
  else if (error == 2)
    Serial.println("Controller found but not accepting commands.");
  else if (error == 3)
    Serial.println("Controller refusing Pressures mode, may not support it.");
}

void detectControllerType() {

  controllerType = ps2x.readType();
  switch (controllerType) {
    case 0:
      Serial.println("Unknown Controller type found");
      break;
    case 1:
      Serial.println("DualShock Controller found");
      break;
    case 2:
      Serial.println("GuitarHero Controller found");
      break;
    case 3:
      Serial.println("Wireless Sony DualShock Controller found");
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

int getStickDeflectionMagnitude(int stickValue) {
  return abs(stickValue - STICK_CENTER);
}

int getExpoDeflectionMagnitude(int linearMagnitude, uint8_t expoPercent) {
  int clampedMagnitude = constrain(linearMagnitude, 0, DRONE_STICK_MAX_DEFLECTION);
  int clampedExpoPercent = constrain(static_cast<int>(expoPercent), 0, 100);
  int quadraticComponent = (clampedMagnitude * clampedMagnitude) / DRONE_STICK_MAX_DEFLECTION;
  int blendedMagnitude = ((100 - clampedExpoPercent) * clampedMagnitude + clampedExpoPercent * quadraticComponent) / 100;
  return blendedMagnitude;
}

uint8_t getProportionalSpeedTier(int magnitude, uint8_t expoPercent) {
  int expoMagnitude = getExpoDeflectionMagnitude(magnitude, expoPercent);

  if (expoMagnitude < DRONE_SPEED_TIER_MED_THRESHOLD) {
    return DRONE_SPEED_TIER_STOP;
  } else if (expoMagnitude < DRONE_SPEED_TIER_HIGH_THRESHOLD) {
    return DRONE_SPEED_TIER_MED;
  }

  return DRONE_SPEED_TIER_HIGH;
}

void applySpeedPinsForTier(uint8_t speedTier, uint8_t upPin, uint8_t downPin) {
  if (speedTier == DRONE_SPEED_TIER_STOP) {
    digitalWrite(upPin, LOW);
    digitalWrite(downPin, LOW);
  } else if (speedTier == DRONE_SPEED_TIER_MED) {
    digitalWrite(upPin, LOW);
    digitalWrite(downPin, HIGH);
  } else {
    digitalWrite(upPin, HIGH);
    digitalWrite(downPin, LOW);
  }
}

uint8_t applyDroneSpeedTierModifiers(uint8_t speedTier, uint8_t maxAllowedTier) {
  bool precisionModeActive = DRONE_ENABLE_PRECISION_MODIFIER && ps2x.Button(PSB_L2);
  bool boostModeActive = DRONE_ENABLE_BOOST_MODIFIER && ps2x.Button(PSB_R2);

  if (DRONE_L2_R2_NEUTRAL_MODE && precisionModeActive && boostModeActive) {
    return speedTier;
  }

  if (precisionModeActive && speedTier > DRONE_SPEED_TIER_STOP) {
    speedTier--;
  }

  if (boostModeActive && speedTier < maxAllowedTier) {
    speedTier++;
  }

  return speedTier;
}

void applyProportionalSpeedPins(int magnitude, uint8_t upPin, uint8_t downPin, uint8_t maxSpeedTier, uint8_t expoPercent) {
  uint8_t speedTier = getProportionalSpeedTier(magnitude, expoPercent);
  uint8_t clampedMaxTier = static_cast<uint8_t>(constrain(static_cast<int>(maxSpeedTier), DRONE_SPEED_TIER_STOP, DRONE_SPEED_TIER_HIGH));

  if (speedTier > clampedMaxTier) {
    speedTier = clampedMaxTier;
  }

  speedTier = applyDroneSpeedTierModifiers(speedTier, clampedMaxTier);

  applySpeedPinsForTier(speedTier, upPin, downPin);
}

void startDroneModeEnterRumbleFeedback() {
  startFeedbackRumble(DRONE_MODE_ENTER_RUMBLE_PULSES, DRONE_MODE_ENTER_RUMBLE_ON_MS, DRONE_MODE_ENTER_RUMBLE_TOTAL_MS);
}

void startDroneModeExitRumbleFeedback() {
  startFeedbackRumble(DRONE_MODE_EXIT_RUMBLE_PULSES, DRONE_MODE_EXIT_RUMBLE_ON_MS, DRONE_MODE_EXIT_RUMBLE_TOTAL_MS);
}

float getFlowlapseTierRateUnitsPerSecond(uint8_t tier) {
  if (tier == DRONE_SPEED_TIER_MED) {
    return FLOWLAPSE_MED_RATE_UNITS_PER_SEC;
  }
  if (tier == DRONE_SPEED_TIER_HIGH) {
    return FLOWLAPSE_HIGH_RATE_UNITS_PER_SEC;
  }
  return 0.0f;
}

void resetFlowlapseAxisTierState(unsigned long now) {
  flowlapseSwingTier = DRONE_SPEED_TIER_STOP;
  flowlapseLiftTier = DRONE_SPEED_TIER_STOP;
  flowlapsePanTier = DRONE_SPEED_TIER_STOP;
  flowlapseTiltTier = DRONE_SPEED_TIER_STOP;

  flowlapseSwingTierLastChangeMs = now;
  flowlapseLiftTierLastChangeMs = now;
  flowlapsePanTierLastChangeMs = now;
  flowlapseTiltTierLastChangeMs = now;
}

void resetFlowlapseSession(bool resetEstimatedPosition) {
  flowlapseWaypointCount = 0;
  flowlapseState = FLOWLAPSE_STATE_RECORDING;
  flowlapseCapturePhase = FLOWLAPSE_CAPTURE_TRIGGER_LOW;
  flowlapseCapturePhaseStartMs = 0;
  flowlapsePreviewHoldUntilMs = 0;
  flowlapseTargetWaypointIndex = 0;
  flowlapseCaptureAlignedToFirstWaypoint = false;
  flowlapseLastUpdateMs = millis();
  flowlapseLastProgressLogMs = 0;
  resetFlowlapseAxisTierState(flowlapseLastUpdateMs);
  digitalWrite(trigger, HIGH);

  if (resetEstimatedPosition) {
    flowlapseCurrentSwingPos = 0.0f;
    flowlapseCurrentLiftPos = 0.0f;
    flowlapseCurrentPanPos = 0.0f;
    flowlapseCurrentTiltPos = 0.0f;
  }
}

const char* getFlowlapseCapturePhaseLabel(FlowlapseCapturePhase phase) {
  switch (phase) {
    case FLOWLAPSE_CAPTURE_TRIGGER_LOW:
      return "trigger-low";
    case FLOWLAPSE_CAPTURE_TRIGGER_HIGH:
      return "trigger-high";
    case FLOWLAPSE_CAPTURE_MOVE_ACTIVE:
      return "move";
    default:
      return "unknown";
  }
}

void logFlowlapseCaptureProgressIfDue(unsigned long now) {
  if (!DRONE_SERIAL_LOG_ENABLED || flowlapseState != FLOWLAPSE_STATE_CAPTURE_RUNNING) {
    return;
  }

  if (flowlapseLastProgressLogMs != 0 && now - flowlapseLastProgressLogMs < FLOWLAPSE_CAPTURE_PROGRESS_LOG_MS) {
    return;
  }

  flowlapseLastProgressLogMs = now;

  uint8_t totalSegments = (flowlapseWaypointCount > 1) ? static_cast<uint8_t>(flowlapseWaypointCount - 1) : 0;
  uint8_t completedSegments = 0;
  if (flowlapseCaptureAlignedToFirstWaypoint && flowlapseTargetWaypointIndex > 0) {
    completedSegments = static_cast<uint8_t>(flowlapseTargetWaypointIndex - 1);
    if (completedSegments > totalSegments) {
      completedSegments = totalSegments;
    }
  }

  uint8_t progressPercent = 0;
  if (totalSegments > 0) {
    progressPercent = static_cast<uint8_t>((static_cast<unsigned long>(completedSegments) * 100UL) / totalSegments);
  }

  Serial.print("Flowlapse capture | waypoint ");
  Serial.print(flowlapseTargetWaypointIndex + 1);
  Serial.print("/");
  Serial.print(flowlapseWaypointCount);
  Serial.print(" phase=");
  Serial.print(getFlowlapseCapturePhaseLabel(flowlapseCapturePhase));
  Serial.print(" progress=");
  Serial.print(progressPercent);
  Serial.println("%");
}

float getFlowlapseDeltaSeconds(unsigned long now) {
  if (flowlapseLastUpdateMs == 0 || now <= flowlapseLastUpdateMs) {
    flowlapseLastUpdateMs = now;
    return 0.0f;
  }

  unsigned long deltaMs = now - flowlapseLastUpdateMs;
  flowlapseLastUpdateMs = now;
  return static_cast<float>(deltaMs) / 1000.0f;
}

void updateFlowlapseEstimatedAxisFromStick(int stickValue, bool isAxisReversed, int axisDeadband, float& axisPosition, float deltaSeconds) {
  int signedOffset = stickValue - STICK_CENTER;
  if (abs(signedOffset) <= axisDeadband || deltaSeconds <= 0.0f) {
    return;
  }

  float normalizedOffset = static_cast<float>(signedOffset) / static_cast<float>(DRONE_STICK_MAX_DEFLECTION);
  float physicalSignedRate = normalizedOffset;
  if (isAxisReversed) {
    physicalSignedRate *= -1.0f;
  }

  axisPosition += physicalSignedRate * FLOWLAPSE_MANUAL_TRACK_RATE_UNITS_PER_SEC * deltaSeconds;
}

void updateFlowlapseEstimatedPositionFromManualSticks(float deltaSeconds) {
  updateFlowlapseEstimatedAxisFromStick(leftStickXvalue, isSwingReversed, DRONE_SWING_DEADBAND, flowlapseCurrentSwingPos, deltaSeconds);
  updateFlowlapseEstimatedAxisFromStick(leftStickYvalue, isLiftReversed, DRONE_LIFT_DEADBAND, flowlapseCurrentLiftPos, deltaSeconds);
  updateFlowlapseEstimatedAxisFromStick(rightStickXvalue, isPanReversed, DRONE_PAN_DEADBAND, flowlapseCurrentPanPos, deltaSeconds);
  updateFlowlapseEstimatedAxisFromStick(rightStickYvalue, isTiltReversed, DRONE_TILT_DEADBAND, flowlapseCurrentTiltPos, deltaSeconds);
}

void captureFlowlapseWaypoint() {
  if (flowlapseWaypointCount >= FLOWLAPSE_MAX_WAYPOINTS) {
    startLimitReachedRumbleFeedback();
    Serial.println("Flowlapse: waypoint limit reached (8).");
    return;
  }

  FlowlapseWaypoint candidateWaypoint;
  candidateWaypoint.swing = flowlapseCurrentSwingPos;
  candidateWaypoint.lift = flowlapseCurrentLiftPos;
  candidateWaypoint.pan = flowlapseCurrentPanPos;
  candidateWaypoint.tilt = flowlapseCurrentTiltPos;

  if (flowlapseWaypointCount > 0) {
    const FlowlapseWaypoint& previousWaypoint = flowlapseWaypoints[flowlapseWaypointCount - 1];
    float deltaSwing = candidateWaypoint.swing - previousWaypoint.swing;
    float deltaLift = candidateWaypoint.lift - previousWaypoint.lift;
    float deltaPan = candidateWaypoint.pan - previousWaypoint.pan;
    float deltaTilt = candidateWaypoint.tilt - previousWaypoint.tilt;
    float separation = sqrt(deltaSwing * deltaSwing + deltaLift * deltaLift + deltaPan * deltaPan + deltaTilt * deltaTilt);

    if (separation < FLOWLAPSE_MIN_WAYPOINT_SEPARATION) {
      startLockoutDeniedRumbleFeedback();
      Serial.println("Flowlapse: waypoint too close to previous point; move rig farther before recording.");
      return;
    }
  }

  FlowlapseWaypoint& waypoint = flowlapseWaypoints[flowlapseWaypointCount];
  waypoint = candidateWaypoint;
  flowlapseWaypointCount++;

  startFeedbackRumble(FLOWLAPSE_WAYPOINT_RUMBLE_PULSES, FLOWLAPSE_WAYPOINT_RUMBLE_ON_MS, FLOWLAPSE_WAYPOINT_RUMBLE_TOTAL_MS);
  Serial.print("Flowlapse: waypoint recorded ");
  Serial.print(flowlapseWaypointCount);
  Serial.print("/");
  Serial.println(FLOWLAPSE_MAX_WAYPOINTS);
}

bool isFlowlapseTargetReached(const FlowlapseWaypoint& target) {
  return fabs(target.swing - flowlapseCurrentSwingPos) <= FLOWLAPSE_AXIS_STOP_TOLERANCE
      && fabs(target.lift  - flowlapseCurrentLiftPos)  <= FLOWLAPSE_AXIS_STOP_TOLERANCE
      && fabs(target.pan   - flowlapseCurrentPanPos)   <= FLOWLAPSE_AXIS_STOP_TOLERANCE
      && fabs(target.tilt  - flowlapseCurrentTiltPos)  <= FLOWLAPSE_AXIS_STOP_TOLERANCE;
}

void applyFlowlapseAxisTowardTarget(float targetPosition, float& estimatedPosition,
                                    bool isAxisReversed,
                                    uint8_t negativeDirectionPin, uint8_t positiveDirectionPin,
                                    uint8_t speedUpPin, uint8_t speedDownPin,
                                    uint8_t& currentTier, unsigned long& tierLastChangeMs,
                                    unsigned long now, float deltaSeconds) {
  digitalWrite(negativeDirectionPin, LOW);
  digitalWrite(positiveDirectionPin, LOW);

  float signedError = targetPosition - estimatedPosition;
  float absError = fabs(signedError);

  uint8_t desiredTier = DRONE_SPEED_TIER_STOP;
  if (absError > FLOWLAPSE_AXIS_MED_ERROR) {
    desiredTier = DRONE_SPEED_TIER_MED;
  }
  if (absError > FLOWLAPSE_AXIS_HIGH_ERROR && FLOWLAPSE_MAX_SPEED_TIER >= DRONE_SPEED_TIER_HIGH) {
    desiredTier = DRONE_SPEED_TIER_HIGH;
  }

  if (desiredTier > FLOWLAPSE_MAX_SPEED_TIER) {
    desiredTier = FLOWLAPSE_MAX_SPEED_TIER;
  }

  if (now - tierLastChangeMs >= FLOWLAPSE_TIER_RAMP_INTERVAL_MS) {
    if (currentTier < desiredTier) {
      currentTier++;
      tierLastChangeMs = now;
    } else if (currentTier > desiredTier) {
      currentTier--;
      tierLastChangeMs = now;
    }
  }

  if (currentTier == DRONE_SPEED_TIER_STOP) {
    applySpeedPinsForTier(DRONE_SPEED_TIER_STOP, speedUpPin, speedDownPin);
    return;
  }

  if (signedError < 0.0f) {
    setDirectionalOutput(isAxisReversed, negativeDirectionPin, positiveDirectionPin, HIGH);
  } else {
    setDirectionalOutput(isAxisReversed, positiveDirectionPin, negativeDirectionPin, HIGH);
  }

  applySpeedPinsForTier(currentTier, speedUpPin, speedDownPin);

  float signedRate = getFlowlapseTierRateUnitsPerSecond(currentTier);
  if (signedError < 0.0f) {
    signedRate *= -1.0f;
  }

  estimatedPosition += signedRate * deltaSeconds;

  if ((signedError > 0.0f && estimatedPosition > targetPosition)
      || (signedError < 0.0f && estimatedPosition < targetPosition)) {
    estimatedPosition = targetPosition;
  }
}

void applyFlowlapseMotionTowardWaypoint(const FlowlapseWaypoint& target, unsigned long now, float deltaSeconds) {
  applyFlowlapseAxisTowardTarget(target.swing, flowlapseCurrentSwingPos, isSwingReversed,
                                 swingLeft, swingRight, swingSpeedUp, swingSpeedDown,
                                 flowlapseSwingTier, flowlapseSwingTierLastChangeMs,
                                 now, deltaSeconds);

  applyFlowlapseAxisTowardTarget(target.lift, flowlapseCurrentLiftPos, isLiftReversed,
                                 liftUp, liftDown, liftSpeedUp, liftSpeedDown,
                                 flowlapseLiftTier, flowlapseLiftTierLastChangeMs,
                                 now, deltaSeconds);

  applyFlowlapseAxisTowardTarget(target.pan, flowlapseCurrentPanPos, isPanReversed,
                                 panLeft, panRight, panSpeedUp, panSpeedDown,
                                 flowlapsePanTier, flowlapsePanTierLastChangeMs,
                                 now, deltaSeconds);

  applyFlowlapseAxisTowardTarget(target.tilt, flowlapseCurrentTiltPos, isTiltReversed,
                                 tiltUp, tiltDown, tiltSpeedUp, tiltSpeedDown,
                                 flowlapseTiltTier, flowlapseTiltTierLastChangeMs,
                                 now, deltaSeconds);

  digitalWrite(panSpeedUpOnly, LOW);
  digitalWrite(panSpeedDownOnly, LOW);
  digitalWrite(tiltSpeedUpOnly, LOW);
  digitalWrite(tiltSpeedDownOnly, LOW);
}

void startFlowlapsePreview() {
  if (flowlapseWaypointCount < 2) {
    startLockoutDeniedRumbleFeedback();
    Serial.println("Flowlapse: need at least 2 waypoints for preview.");
    return;
  }

  flowlapseState = FLOWLAPSE_STATE_PREVIEW_RUNNING;
  flowlapseTargetWaypointIndex = 0;
  flowlapsePreviewHoldUntilMs = 0;
  resetFlowlapseAxisTierState(millis());
  Serial.println("Flowlapse: preview started.");
}

void startFlowlapseCapture(unsigned long now) {
  if (flowlapseWaypointCount < 2) {
    startLockoutDeniedRumbleFeedback();
    Serial.println("Flowlapse: need at least 2 waypoints to run capture.");
    return;
  }

  flowlapseState = FLOWLAPSE_STATE_CAPTURE_RUNNING;
  flowlapseTargetWaypointIndex = 0;
  flowlapseCaptureAlignedToFirstWaypoint = false;
  flowlapseCapturePhase = FLOWLAPSE_CAPTURE_TRIGGER_LOW;
  flowlapseCapturePhaseStartMs = now;
  flowlapseLastProgressLogMs = 0;
  resetFlowlapseAxisTierState(now);
  Serial.println("Flowlapse: capture run started.");
}

void enterDroneMode() {
  resetTimelapseState();
  resetBounceState();
  stopIntervalRumbleFeedback();
  resetFlowlapseSession(true);
  droneMode = true;
  droneLastActivityMs = millis();
  Serial.println("DRONE MODE ACTIVATED - timelapse/bounce locked out");
  Serial.println("Flowlapse: recording armed. L3=record waypoint, SELECT=stop record.");
  startDroneModeEnterRumbleFeedback();
}

void exitDroneMode() {
  droneMode = false;
  lastDronePrecisionModeActive = false;
  lastDroneBoostModeActive = false;
  lastDroneSwingActive = false;
  lastDroneLiftActive = false;
  lastDronePanActive = false;
  lastDroneTiltActive = false;
  resetFlowlapseSession(true);
  droneLastActivityMs = 0;
  stopAllMotors();
  Serial.println("DRONE MODE DEACTIVATED");
  startDroneModeExitRumbleFeedback();
}

void logDroneAxisStateIfChanged(bool current, bool& last, const char* axisName) {
  if (current != last) {
    if (DRONE_SERIAL_LOG_ENABLED) {
      Serial.print("Drone axis | ");
      Serial.print(axisName);
      Serial.println(current ? " MOVING" : " STOPPED");
    }
    last = current;
  }
}

void logDroneSpeedModifierStateIfChanged() {
  bool precisionModeActive = DRONE_ENABLE_PRECISION_MODIFIER && ps2x.Button(PSB_L2);
  bool boostModeActive = DRONE_ENABLE_BOOST_MODIFIER && ps2x.Button(PSB_R2);

  if (precisionModeActive != lastDronePrecisionModeActive || boostModeActive != lastDroneBoostModeActive) {
    if (DRONE_SERIAL_LOG_ENABLED) {
      Serial.print("Drone speed modifier | precision=");
      Serial.print(precisionModeActive ? "ON" : "OFF");
      Serial.print(" boost=");
      Serial.println(boostModeActive ? "ON" : "OFF");
    }
    lastDronePrecisionModeActive = precisionModeActive;
    lastDroneBoostModeActive = boostModeActive;
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
                                    int& soloState, bool& motionState) {
  if (soloState == 0 && ps2x.Button(buttonCode)) {
    if (DEBUG_EDGE_EVENTS && (buttonCode == PSB_PAD_LEFT || buttonCode == PSB_PAD_RIGHT)) {
      Serial.print("COMBINED press: ");
      Serial.println(buttonCode == PSB_PAD_LEFT ? "PAD_LEFT" : "PAD_RIGHT");
    }
    motionState = true;
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
    motionState = false;
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
  if (swingInMotion) {
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
  if (!swingInMotion && panStop == PAN_STOP_ACTIVE && rightStickXvalue == STICK_CENTER) {
    digitalWrite(panLeft, LOW);
    digitalWrite(panRight, LOW);
    digitalWrite(panSpeedUpOnly, LOW);
    digitalWrite(panSpeedDownOnly, LOW);
    panStop = PAN_STOP_NONE;
  }
}

void handlePanAxis() {
  if (!swingInMotion) {
    if (rightStickXvalue == STICK_MIN) {
      activatePanOnlyMotion(panLeft, panRight, "pan left only with top speed");
    } else if (rightStickXvalue == STICK_MAX) {
      activatePanOnlyMotion(panRight, panLeft, "pan right only with top speed");
    }
  }

  stopPanOnlyMotionAtCenter();
}

void handleLiftOnly(uint8_t buttonCode, uint8_t liftNormalPin, uint8_t liftReversedPin) {
  handleSoloDirectionalMode(buttonCode, isLiftReversed, liftNormalPin, liftReversedPin, liftSoloMode);
}

void handleTiltTrimAxis() {
  if (liftInMotion && rightStickYvalue == STICK_MAX) {
    Serial.println("tiltSpeedDownOnly");
    digitalWrite(tiltSpeedDownOnly, HIGH);
    tiltStop = TILT_STOP_TRIM;
  }
  if (liftInMotion && rightStickYvalue == STICK_MIN) {
    Serial.println("tiltSpeedUpOnly");
    digitalWrite(tiltSpeedUpOnly, HIGH);
    tiltStop = TILT_STOP_TRIM;
  }
  if (tiltStop == TILT_STOP_TRIM && rightStickYvalue == STICK_CENTER) {
    digitalWrite(tiltSpeedUpOnly, LOW);
    digitalWrite(tiltSpeedDownOnly, LOW);
    tiltStop = TILT_STOP_NONE;
  }
}

void handleLiftAndTilt(uint8_t buttonCode, uint8_t liftNormalPin, uint8_t liftReversedPin,
                       uint8_t tiltNormalPin, uint8_t tiltReversedPin) {
  handleCombinedDirectionalMode(buttonCode, isLiftReversed, liftNormalPin, liftReversedPin,
                                isTiltReversed, tiltNormalPin, tiltReversedPin, liftSoloMode, liftInMotion);
}

void activateTiltOnlyMotion(uint8_t normalPin, uint8_t reversedPin, const char* label) {
  Serial.println(label);
  digitalWrite(tiltSpeedUpOnly, HIGH);
  digitalWrite(tiltSpeedDownOnly, HIGH);
  setDirectionalOutput(isTiltReversed, normalPin, reversedPin, HIGH);
  tiltStop = TILT_STOP_ACTIVE;
}

void stopTiltOnlyMotionAtCenter() {
  if (!liftInMotion && tiltStop == TILT_STOP_ACTIVE && rightStickYvalue == STICK_CENTER) {
    digitalWrite(tiltUp, LOW);
    digitalWrite(tiltDown, LOW);
    digitalWrite(tiltSpeedUpOnly, LOW);
    digitalWrite(tiltSpeedDownOnly, LOW);
    tiltStop = TILT_STOP_NONE;
  }
}

void handleTiltAxis() {
  if (!liftInMotion) {
    if (rightStickYvalue == STICK_MAX) {
      activateTiltOnlyMotion(tiltDown, tiltUp, "tiltDownOnly");
    } else if (rightStickYvalue == STICK_MIN) {
      activateTiltOnlyMotion(tiltUp, tiltDown, "tiltUpOnly");
    }
  }

  stopTiltOnlyMotionAtCenter();
}

void handleButtonDirection(uint8_t buttonCode, bool isReversed,
                           uint8_t normalPin, uint8_t reversedPin) {
  if (ps2x.Button(buttonCode)) {
    setDirectionalOutput(isReversed, normalPin, reversedPin, HIGH);
  }
  if (ps2x.ButtonReleased(buttonCode)) {
    digitalWrite(normalPin, LOW);
    digitalWrite(reversedPin, LOW);
  }
}

void handleFocusAxis() {
  handleButtonDirection(PSB_TRIANGLE, isFocusReversed, focusLeft, focusRight);
  handleButtonDirection(PSB_CROSS, isFocusReversed, focusRight, focusLeft);
  if (ps2x.Button(PSB_SQUARE))         digitalWrite(focusSpeedDown, HIGH);
  if (ps2x.ButtonReleased(PSB_SQUARE)) digitalWrite(focusSpeedDown, LOW);
  if (ps2x.Button(PSB_CIRCLE))         digitalWrite(focusSpeedUp, HIGH);
  if (ps2x.ButtonReleased(PSB_CIRCLE)) digitalWrite(focusSpeedUp, LOW);
}

// Returns true if the axis is active (outside deadband), false if stopped.
bool applyDroneAxisControl(int stickValue, bool isReversed,
                           uint8_t negativeDirectionPin, uint8_t positiveDirectionPin,
                           uint8_t speedUpPin, uint8_t speedDownPin,
                           int axisDeadband, uint8_t maxSpeedTier, uint8_t expoPercent) {
  digitalWrite(negativeDirectionPin, LOW);
  digitalWrite(positiveDirectionPin, LOW);

  int signedOffsetFromCenter = stickValue - STICK_CENTER;
  if (abs(signedOffsetFromCenter) <= axisDeadband) {
    applySpeedPinsForTier(DRONE_SPEED_TIER_STOP, speedUpPin, speedDownPin);
    return false;
  }

  int magnitude = getStickDeflectionMagnitude(stickValue);
  applyProportionalSpeedPins(magnitude, speedUpPin, speedDownPin, maxSpeedTier, expoPercent);

  if (stickValue < STICK_CENTER - axisDeadband) {
    setDirectionalOutput(isReversed, negativeDirectionPin, positiveDirectionPin, HIGH);
  } else if (stickValue > STICK_CENTER + axisDeadband) {
    setDirectionalOutput(isReversed, positiveDirectionPin, negativeDirectionPin, HIGH);
  }
  return true;
}

void handleDroneStickControl() {
  bool anyAxisActive =
    abs(leftStickXvalue  - STICK_CENTER) > DRONE_SWING_DEADBAND ||
    abs(leftStickYvalue  - STICK_CENTER) > DRONE_LIFT_DEADBAND  ||
    abs(rightStickXvalue - STICK_CENTER) > DRONE_PAN_DEADBAND   ||
    abs(rightStickYvalue - STICK_CENTER) > DRONE_TILT_DEADBAND;

  if (anyAxisActive) {
    droneLastActivityMs = millis();
  }

  logDroneSpeedModifierStateIfChanged();

  // Left stick controls swing (X) and lift (Y)
  bool swingActive = applyDroneAxisControl(leftStickXvalue, isSwingReversed, swingLeft, swingRight, swingSpeedUp, swingSpeedDown, DRONE_SWING_DEADBAND, DRONE_SWING_MAX_SPEED_TIER, DRONE_SWING_EXPO_PERCENT);
  bool liftActive  = applyDroneAxisControl(leftStickYvalue, isLiftReversed, liftUp, liftDown, liftSpeedUp, liftSpeedDown, DRONE_LIFT_DEADBAND, DRONE_LIFT_MAX_SPEED_TIER, DRONE_LIFT_EXPO_PERCENT);

  // Right stick controls pan (X) and tilt (Y)
  bool panActive  = applyDroneAxisControl(rightStickXvalue, isPanReversed, panLeft, panRight, panSpeedUp, panSpeedDown, DRONE_PAN_DEADBAND, DRONE_PAN_MAX_SPEED_TIER, DRONE_PAN_EXPO_PERCENT);
  bool tiltActive = applyDroneAxisControl(rightStickYvalue, isTiltReversed, tiltUp, tiltDown, tiltSpeedUp, tiltSpeedDown, DRONE_TILT_DEADBAND, DRONE_TILT_MAX_SPEED_TIER, DRONE_TILT_EXPO_PERCENT);

  logDroneAxisStateIfChanged(swingActive, lastDroneSwingActive, "swing");
  logDroneAxisStateIfChanged(liftActive,  lastDroneLiftActive,  "lift");
  logDroneAxisStateIfChanged(panActive,   lastDronePanActive,   "pan");
  logDroneAxisStateIfChanged(tiltActive,  lastDroneTiltActive,  "tilt");

  // Disable trim-only speed pins in drone mode
  digitalWrite(panSpeedUpOnly, LOW);
  digitalWrite(panSpeedDownOnly, LOW);
  digitalWrite(tiltSpeedUpOnly, LOW);
  digitalWrite(tiltSpeedDownOnly, LOW);
}

void completeFlowlapsePreview() {
  stopAllMotors();
  flowlapseState = FLOWLAPSE_STATE_READY_FOR_CAPTURE;
  flowlapseTargetWaypointIndex = 0;
  flowlapsePreviewHoldUntilMs = 0;
  Serial.println("Flowlapse: preview complete. Press START to run capture.");
}

void completeFlowlapseCapture() {
  stopAllMotors();
  digitalWrite(trigger, HIGH);
  flowlapseState = FLOWLAPSE_STATE_READY_FOR_CAPTURE;
  flowlapseCapturePhase = FLOWLAPSE_CAPTURE_TRIGGER_LOW;
  flowlapseCapturePhaseStartMs = 0;
  flowlapseCaptureAlignedToFirstWaypoint = false;
  flowlapseTargetWaypointIndex = 0;
  Serial.println("Flowlapse: capture complete.");
}

void handleFlowlapsePreviewStep(unsigned long now, float deltaSeconds) {
  if (flowlapseTargetWaypointIndex >= flowlapseWaypointCount) {
    completeFlowlapsePreview();
    return;
  }

  if (flowlapsePreviewHoldUntilMs != 0) {
    stopAllMotors();
    if (now < flowlapsePreviewHoldUntilMs) {
      return;
    }

    flowlapsePreviewHoldUntilMs = 0;
    flowlapseTargetWaypointIndex++;
    if (flowlapseTargetWaypointIndex >= flowlapseWaypointCount) {
      completeFlowlapsePreview();
      return;
    }
  }

  const FlowlapseWaypoint& target = flowlapseWaypoints[flowlapseTargetWaypointIndex];
  applyFlowlapseMotionTowardWaypoint(target, now, deltaSeconds);

  if (isFlowlapseTargetReached(target)) {
    stopAllMotors();
    flowlapsePreviewHoldUntilMs = now + FLOWLAPSE_PREVIEW_POINT_HOLD_MS;
    Serial.print("Flowlapse preview reached waypoint ");
    Serial.println(flowlapseTargetWaypointIndex + 1);
  }
}

void handleFlowlapseCaptureStep(unsigned long now, float deltaSeconds) {
  logFlowlapseCaptureProgressIfDue(now);

  if (flowlapseWaypointCount < 2) {
    completeFlowlapseCapture();
    return;
  }

  if (!flowlapseCaptureAlignedToFirstWaypoint) {
    const FlowlapseWaypoint& firstWaypoint = flowlapseWaypoints[0];
    applyFlowlapseMotionTowardWaypoint(firstWaypoint, now, deltaSeconds);

    if (isFlowlapseTargetReached(firstWaypoint)) {
      stopAllMotors();
      flowlapseCaptureAlignedToFirstWaypoint = true;
      flowlapseCapturePhase = FLOWLAPSE_CAPTURE_TRIGGER_LOW;
      flowlapseCapturePhaseStartMs = now;
      flowlapseTargetWaypointIndex = 1;
      Serial.println("Flowlapse: aligned to first waypoint. Capture triggering started.");
    }
    return;
  }

  switch (flowlapseCapturePhase) {
    case FLOWLAPSE_CAPTURE_TRIGGER_LOW:
      stopAllMotors();
      digitalWrite(trigger, LOW);
      if (now - flowlapseCapturePhaseStartMs >= static_cast<unsigned long>(timelapseIntervalMs / 2)) {
        digitalWrite(trigger, HIGH);
        flowlapseCapturePhase = FLOWLAPSE_CAPTURE_TRIGGER_HIGH;
        flowlapseCapturePhaseStartMs = now;
      }
      break;

    case FLOWLAPSE_CAPTURE_TRIGGER_HIGH:
      stopAllMotors();
      digitalWrite(trigger, HIGH);
      if (now - flowlapseCapturePhaseStartMs >= static_cast<unsigned long>(timelapseIntervalMs / 2)) {
        flowlapseCapturePhase = FLOWLAPSE_CAPTURE_MOVE_ACTIVE;
        flowlapseCapturePhaseStartMs = now;
      }
      break;

    case FLOWLAPSE_CAPTURE_MOVE_ACTIVE:
      if (flowlapseTargetWaypointIndex >= flowlapseWaypointCount) {
        completeFlowlapseCapture();
        return;
      }

      applyFlowlapseMotionTowardWaypoint(flowlapseWaypoints[flowlapseTargetWaypointIndex], now, deltaSeconds);

      if (isFlowlapseTargetReached(flowlapseWaypoints[flowlapseTargetWaypointIndex])) {
        flowlapseTargetWaypointIndex++;
        if (flowlapseTargetWaypointIndex >= flowlapseWaypointCount) {
          completeFlowlapseCapture();
          return;
        }
      }

      if (now - flowlapseCapturePhaseStartMs >= static_cast<unsigned long>(stepDist)) {
        stopAllMotors();
        flowlapseCapturePhase = FLOWLAPSE_CAPTURE_TRIGGER_LOW;
        flowlapseCapturePhaseStartMs = now;
      }
      break;
  }
}

void handleDroneFlowlapseButtons(unsigned long now) {
  bool flowlapseClearComboActive = ps2x.Button(PSB_L1) && ps2x.Button(PSB_R1);
  if (flowlapseClearComboActive && !lastFlowlapseClearComboActive) {
    resetFlowlapseSession(false);
    stopAllMotors();
    startFeedbackRumble(2, FLOWLAPSE_WAYPOINT_RUMBLE_ON_MS, FLOWLAPSE_WAYPOINT_RUMBLE_TOTAL_MS);
    Serial.println("Flowlapse: waypoints cleared. Recording re-armed.");
    droneLastActivityMs = now;
  }
  lastFlowlapseClearComboActive = flowlapseClearComboActive;

  if (flowlapseState == FLOWLAPSE_STATE_RECORDING && ps2x.ButtonReleased(PSB_L3)) {
    captureFlowlapseWaypoint();
    droneLastActivityMs = now;
  }

  if (ps2x.ButtonReleased(PSB_SELECT)) {
    if (flowlapseState == FLOWLAPSE_STATE_RECORDING) {
      if (flowlapseWaypointCount < 2) {
        startLockoutDeniedRumbleFeedback();
        Serial.println("Flowlapse: record at least 2 waypoints before stopping record.");
      } else {
        flowlapseState = FLOWLAPSE_STATE_READY_FOR_PREVIEW;
        stopAllMotors();
        Serial.println("Flowlapse: recording stopped. Press SELECT again for preview.");
      }
      droneLastActivityMs = now;
    } else if (flowlapseState == FLOWLAPSE_STATE_READY_FOR_PREVIEW || flowlapseState == FLOWLAPSE_STATE_READY_FOR_CAPTURE) {
      startFlowlapsePreview();
      droneLastActivityMs = now;
    }
  }

  if (ps2x.ButtonReleased(PSB_START)) {
    if (flowlapseState == FLOWLAPSE_STATE_READY_FOR_CAPTURE) {
      startFlowlapseCapture(now);
      droneLastActivityMs = now;
    } else if (flowlapseState == FLOWLAPSE_STATE_READY_FOR_PREVIEW) {
      startLockoutDeniedRumbleFeedback();
      Serial.println("Flowlapse: run preview first (SELECT), then press START for capture.");
      droneLastActivityMs = now;
    }
  }
}

void handleDroneFlowlapseWorkflow(unsigned long now, float deltaSeconds) {
  handleDroneFlowlapseButtons(now);

  if (flowlapseState == FLOWLAPSE_STATE_PREVIEW_RUNNING) {
    handleFlowlapsePreviewStep(now, deltaSeconds);
    droneLastActivityMs = now;
    return;
  }

  if (flowlapseState == FLOWLAPSE_STATE_CAPTURE_RUNNING) {
    handleFlowlapseCaptureStep(now, deltaSeconds);
    droneLastActivityMs = now;
    return;
  }

  if (flowlapseState == FLOWLAPSE_STATE_RECORDING) {
    handleDroneStickControl();
    updateFlowlapseEstimatedPositionFromManualSticks(deltaSeconds);
    handleFocusAxis();
    return;
  }

  stopAllMotors();
}

void stopAllMotors() {
  digitalWrite(swingLeft, LOW);
  digitalWrite(swingRight, LOW);
  digitalWrite(swingSpeedUp, LOW);
  digitalWrite(swingSpeedDown, LOW);
  digitalWrite(panLeft, LOW);
  digitalWrite(panRight, LOW);
  digitalWrite(panSpeedUp, LOW);
  digitalWrite(panSpeedDown, LOW);
  digitalWrite(panSpeedUpOnly, LOW);
  digitalWrite(panSpeedDownOnly, LOW);
  digitalWrite(liftUp, LOW);
  digitalWrite(liftDown, LOW);
  digitalWrite(liftSpeedUp, LOW);
  digitalWrite(liftSpeedDown, LOW);
  digitalWrite(tiltUp, LOW);
  digitalWrite(tiltDown, LOW);
  digitalWrite(tiltSpeedUp, LOW);
  digitalWrite(tiltSpeedDown, LOW);
  digitalWrite(tiltSpeedUpOnly, LOW);
  digitalWrite(tiltSpeedDownOnly, LOW);
  digitalWrite(focusLeft, LOW);
  digitalWrite(focusRight, LOW);
  digitalWrite(focusSpeedUp, LOW);
  digitalWrite(focusSpeedDown, LOW);
}

void updateIntervalMs() {
  timelapseIntervalMs = static_cast<unsigned long>(timelapseIntervalSeconds) * 1000UL;
}

void stopIntervalRumbleFeedback() {
  intervalRumblePhase = INTERVAL_RUMBLE_IDLE;
  intervalRumblePhaseStartMs = 0;
  intervalRumbleLongsRemaining = 0;
  intervalRumbleShortsRemaining = 0;
  stepDistRumblePhase = STEP_DIST_RUMBLE_IDLE;
  stepDistRumblePhaseStartMs = 0;
  stepDistRumbleLongsRemaining = 0;
  pendingRumblePattern = PENDING_RUMBLE_NONE;
  rumbleSeparatorActive = false;
  rumbleSeparatorStartMs = 0;
  feedbackRumblePhase = FEEDBACK_RUMBLE_IDLE;
  feedbackRumblePhaseStartMs = 0;
  feedbackRumblePulsesRemaining = 0;
  feedbackRumbleOnMs = 0;
  feedbackRumbleTotalMs = 0;
  chainStepDistAfterInterval = false;
  vibrate = 0;
}

void startFeedbackRumble(uint8_t pulses, unsigned long onMs, unsigned long totalMs) {
  if (pulses == 0 || totalMs < onMs) {
    stopIntervalRumbleFeedback();
    return;
  }

  stopIntervalRumbleFeedback();
  feedbackRumblePhase = FEEDBACK_RUMBLE_ON;
  feedbackRumblePhaseStartMs = millis();
  feedbackRumblePulsesRemaining = pulses;
  feedbackRumbleOnMs = onMs;
  feedbackRumbleTotalMs = totalMs;
  vibrate = RUMBLE_ACTIVE_INTENSITY;
}

void startLimitReachedRumbleFeedback() {
  startFeedbackRumble(LIMIT_REACHED_RUMBLE_PULSES, FEEDBACK_RUMBLE_ON_MS, FEEDBACK_RUMBLE_TOTAL_MS);
}

void startLockoutDeniedRumbleFeedback() {
  startFeedbackRumble(LOCKOUT_DENIED_RUMBLE_PULSES, FEEDBACK_RUMBLE_ON_MS, FEEDBACK_RUMBLE_TOTAL_MS);
}

void startEmergencyReleaseRumbleFeedback() {
  startFeedbackRumble(EMERGENCY_RELEASE_RUMBLE_PULSES, EMERGENCY_RELEASE_RUMBLE_ON_MS, EMERGENCY_RELEASE_RUMBLE_TOTAL_MS);
}

void startL3EndpointRumbleFeedback() {
  startFeedbackRumble(L3_ENDPOINT_RUMBLE_PULSES, L3_ENDPOINT_RUMBLE_ON_MS, L3_ENDPOINT_RUMBLE_TOTAL_MS);
}

void startRumbleUnmuteFeedback() {
  startFeedbackRumble(1, FEEDBACK_RUMBLE_ON_MS, FEEDBACK_RUMBLE_TOTAL_MS);
}

void startSettingsReplayRumble() {
  chainStepDistAfterInterval = true;
  startIntervalRumbleFeedback();
  Serial.print("Settings replay: interval=");
  Serial.print(timelapseIntervalSeconds);
  Serial.print("s, stepDist=");
  Serial.print(stepDist);
  Serial.println("ms");
}

void startIntervalRumbleFeedbackNow() {
  stepDistRumblePhase = STEP_DIST_RUMBLE_IDLE;
  stepDistRumblePhaseStartMs = 0;
  stepDistRumbleLongsRemaining = 0;

  intervalRumbleLongsRemaining = timelapseIntervalSeconds / 10;
  intervalRumbleShortsRemaining = timelapseIntervalSeconds % 10;

  if (intervalRumbleLongsRemaining > 0) {
    intervalRumblePhase = INTERVAL_RUMBLE_LONG_ACTIVE;
  } else if (intervalRumbleShortsRemaining > 0) {
    intervalRumblePhase = INTERVAL_RUMBLE_SHORT_ACTIVE;
  } else {
    stopIntervalRumbleFeedback();
    return;
  }

  intervalRumblePhaseStartMs = millis();
  vibrate = RUMBLE_ACTIVE_INTENSITY;
}

void startStepDistRumbleFeedbackNow() {
  intervalRumblePhase = INTERVAL_RUMBLE_IDLE;
  intervalRumblePhaseStartMs = 0;
  intervalRumbleLongsRemaining = 0;
  intervalRumbleShortsRemaining = 0;

  stepDistRumbleLongsRemaining = stepDist / TIMELAPSE_STEP_DIST_ADJUST_INCREMENT_MS;
  if (stepDistRumbleLongsRemaining == 0) {
    stopIntervalRumbleFeedback();
    return;
  }

  stepDistRumblePhase = STEP_DIST_RUMBLE_LONG_ACTIVE;
  stepDistRumblePhaseStartMs = millis();
  vibrate = RUMBLE_ACTIVE_INTENSITY;
}

void startRumbleSeparator(PendingRumblePattern nextPattern) {
  intervalRumblePhase = INTERVAL_RUMBLE_IDLE;
  intervalRumblePhaseStartMs = 0;
  intervalRumbleLongsRemaining = 0;
  intervalRumbleShortsRemaining = 0;
  stepDistRumblePhase = STEP_DIST_RUMBLE_IDLE;
  stepDistRumblePhaseStartMs = 0;
  stepDistRumbleLongsRemaining = 0;
  pendingRumblePattern = nextPattern;
  rumbleSeparatorActive = true;
  rumbleSeparatorStartMs = millis();
  vibrate = 0;
}

void startIntervalRumbleFeedback() {
  if (stepDistRumblePhase != STEP_DIST_RUMBLE_IDLE) {
    startRumbleSeparator(PENDING_RUMBLE_INTERVAL);
    return;
  }

  pendingRumblePattern = PENDING_RUMBLE_NONE;
  rumbleSeparatorActive = false;
  rumbleSeparatorStartMs = 0;
  startIntervalRumbleFeedbackNow();
}

void startStepDistRumbleFeedback() {
  if (intervalRumblePhase != INTERVAL_RUMBLE_IDLE) {
    startRumbleSeparator(PENDING_RUMBLE_STEP_DIST);
    return;
  }

  pendingRumblePattern = PENDING_RUMBLE_NONE;
  rumbleSeparatorActive = false;
  rumbleSeparatorStartMs = 0;
  startStepDistRumbleFeedbackNow();
}

void handleIntervalRumbleFeedback(unsigned long now) {
  if (feedbackRumblePhase != FEEDBACK_RUMBLE_IDLE) {
    switch (feedbackRumblePhase) {
      case FEEDBACK_RUMBLE_ON:
        vibrate = RUMBLE_ACTIVE_INTENSITY;
        if (now - feedbackRumblePhaseStartMs >= feedbackRumbleOnMs) {
          feedbackRumblePhase = FEEDBACK_RUMBLE_PAUSE;
          feedbackRumblePhaseStartMs = now;
          vibrate = 0;
        }
        break;
      case FEEDBACK_RUMBLE_PAUSE:
        vibrate = 0;
        if (now - feedbackRumblePhaseStartMs >= (feedbackRumbleTotalMs - feedbackRumbleOnMs)) {
          if (feedbackRumblePulsesRemaining > 0) {
            feedbackRumblePulsesRemaining--;
          }
          if (feedbackRumblePulsesRemaining > 0) {
            feedbackRumblePhase = FEEDBACK_RUMBLE_ON;
            feedbackRumblePhaseStartMs = now;
          } else {
            stopIntervalRumbleFeedback();
          }
        }
        break;
      case FEEDBACK_RUMBLE_IDLE:
        break;
    }
    return;
  }

  if (rumbleSeparatorActive) {
    vibrate = 0;
    if (now - rumbleSeparatorStartMs >= RUMBLE_PATTERN_SEPARATOR_MS) {
      rumbleSeparatorActive = false;
      rumbleSeparatorStartMs = 0;
      PendingRumblePattern patternToStart = pendingRumblePattern;
      pendingRumblePattern = PENDING_RUMBLE_NONE;

      if (patternToStart == PENDING_RUMBLE_INTERVAL) {
        startIntervalRumbleFeedbackNow();
      } else if (patternToStart == PENDING_RUMBLE_STEP_DIST) {
        startStepDistRumbleFeedbackNow();
      }
    }
    return;
  }

  if (stepDistRumblePhase != STEP_DIST_RUMBLE_IDLE) {
    switch (stepDistRumblePhase) {
      case STEP_DIST_RUMBLE_LONG_ACTIVE:
        vibrate = RUMBLE_ACTIVE_INTENSITY;
        if (now - stepDistRumblePhaseStartMs >= STEP_DIST_RUMBLE_LONG_ON_MS) {
          stepDistRumblePhase = STEP_DIST_RUMBLE_LONG_PAUSE;
          stepDistRumblePhaseStartMs = now;
          vibrate = 0;
        }
        break;
      case STEP_DIST_RUMBLE_LONG_PAUSE:
        vibrate = 0;
        if (now - stepDistRumblePhaseStartMs >= (STEP_DIST_RUMBLE_LONG_TOTAL_MS - STEP_DIST_RUMBLE_LONG_ON_MS)) {
          if (stepDistRumbleLongsRemaining > 0) {
            stepDistRumbleLongsRemaining--;
          }
          if (stepDistRumbleLongsRemaining > 0) {
            stepDistRumblePhase = STEP_DIST_RUMBLE_LONG_ACTIVE;
            stepDistRumblePhaseStartMs = now;
          } else {
            stopIntervalRumbleFeedback();
          }
        }
        break;
      case STEP_DIST_RUMBLE_IDLE:
        break;
    }
    return;
  }

  switch (intervalRumblePhase) {
    case INTERVAL_RUMBLE_IDLE:
      vibrate = 0;
      break;
    case INTERVAL_RUMBLE_LONG_ACTIVE:
      vibrate = RUMBLE_ACTIVE_INTENSITY;
      if (now - intervalRumblePhaseStartMs >= INTERVAL_RUMBLE_LONG_MS) {
        intervalRumbleLongsRemaining--;
        if (intervalRumbleLongsRemaining > 0) {
          intervalRumblePhaseStartMs = now;
        } else if (intervalRumbleShortsRemaining > 0) {
          intervalRumblePhase = INTERVAL_RUMBLE_SHORT_ACTIVE;
          intervalRumblePhaseStartMs = now;
        } else {
          if (chainStepDistAfterInterval) {
            chainStepDistAfterInterval = false;
            startRumbleSeparator(PENDING_RUMBLE_STEP_DIST);
          } else {
            stopIntervalRumbleFeedback();
          }
        }
      }
      break;
    case INTERVAL_RUMBLE_SHORT_ACTIVE:
      vibrate = RUMBLE_ACTIVE_INTENSITY;
      if (now - intervalRumblePhaseStartMs >= INTERVAL_RUMBLE_SHORT_ON_MS) {
        intervalRumblePhase = INTERVAL_RUMBLE_SHORT_PAUSE;
        intervalRumblePhaseStartMs = now;
        vibrate = 0;
      }
      break;
    case INTERVAL_RUMBLE_SHORT_PAUSE:
      vibrate = 0;
      if (now - intervalRumblePhaseStartMs >= (INTERVAL_RUMBLE_SHORT_TOTAL_MS - INTERVAL_RUMBLE_SHORT_ON_MS)) {
        intervalRumbleShortsRemaining--;
        if (intervalRumbleShortsRemaining > 0) {
          intervalRumblePhase = INTERVAL_RUMBLE_SHORT_ACTIVE;
          intervalRumblePhaseStartMs = now;
        } else {
          if (chainStepDistAfterInterval) {
            chainStepDistAfterInterval = false;
            startRumbleSeparator(PENDING_RUMBLE_STEP_DIST);
          } else {
            stopIntervalRumbleFeedback();
          }
        }
      }
      break;
  }
}

void adjustIntervalSeconds(int delta) {
  int newIntervalSeconds = timelapseIntervalSeconds + delta;
  if (newIntervalSeconds < TIMELAPSE_INTERVAL_MIN_SECONDS) {
    newIntervalSeconds = TIMELAPSE_INTERVAL_MIN_SECONDS;
  }
  if (newIntervalSeconds > TIMELAPSE_INTERVAL_MAX_SECONDS) {
    newIntervalSeconds = TIMELAPSE_INTERVAL_MAX_SECONDS;
  }
  if (newIntervalSeconds == timelapseIntervalSeconds) {
    startLimitReachedRumbleFeedback();
    Serial.println("Timelapse interval limit reached.");
    return;
  }

  timelapseIntervalSeconds = newIntervalSeconds;
  updateIntervalMs();
  Serial.print("Timelapse interval (seconds) = ");
  Serial.println(timelapseIntervalSeconds);
  startIntervalRumbleFeedback();
}

void adjustStepDist(int delta) {
  int newStepDist = stepDist + delta;
  if (newStepDist < TIMELAPSE_STEP_DIST_MIN_MS) {
    newStepDist = TIMELAPSE_STEP_DIST_MIN_MS;
  }
  if (newStepDist > TIMELAPSE_STEP_DIST_MAX_MS) {
    newStepDist = TIMELAPSE_STEP_DIST_MAX_MS;
  }
  if (newStepDist == stepDist) {
    startLimitReachedRumbleFeedback();
    Serial.println("Timelapse stepDist limit reached.");
    return;
  }

  stepDist = newStepDist;
  Serial.print("Timelapse stepDist (ms) = ");
  Serial.println(stepDist);
  startStepDistRumbleFeedback();
}

bool isAutoModeActive() {
  return (timelapseMode != 0 || bounce != 0);
}

bool isRumbleFeedbackActive() {
  return (intervalRumblePhase != INTERVAL_RUMBLE_IDLE
      || stepDistRumblePhase != STEP_DIST_RUMBLE_IDLE
      || rumbleSeparatorActive
      || feedbackRumblePhase != FEEDBACK_RUMBLE_IDLE
      || pendingRumblePattern != PENDING_RUMBLE_NONE);
}

// START + PAD_UP increases timelapseIntervalSeconds.
// START + PAD_DOWN decreases timelapseIntervalSeconds.
// This is only active while no auto mode is running so it does not conflict
// with active timelapse or bounce motion.
bool handleTimelapseIntervalAdjustment() {
  bool adjustmentAllowed = !isAutoModeActive();
  bool intervalAdjustUpComboRawActive = ps2x.Button(PSB_START) && ps2x.Button(PSB_PAD_UP);
  bool intervalAdjustDownComboRawActive = ps2x.Button(PSB_START) && ps2x.Button(PSB_PAD_DOWN);

  if (intervalAdjustUpComboRawActive && !lastIntervalAdjustUpComboActive) {
    if (adjustmentAllowed) {
      adjustIntervalSeconds(1);
    } else {
      startLockoutDeniedRumbleFeedback();
      Serial.println("Timelapse interval adjustment blocked: auto mode active.");
    }
  }

  if (intervalAdjustDownComboRawActive && !lastIntervalAdjustDownComboActive) {
    if (adjustmentAllowed) {
      adjustIntervalSeconds(-1);
    } else {
      startLockoutDeniedRumbleFeedback();
      Serial.println("Timelapse interval adjustment blocked: auto mode active.");
    }
  }

  lastIntervalAdjustUpComboActive = intervalAdjustUpComboRawActive;
  lastIntervalAdjustDownComboActive = intervalAdjustDownComboRawActive;

  bool intervalAdjustComboHandled = adjustmentAllowed && (intervalAdjustUpComboRawActive || intervalAdjustDownComboRawActive);

  if (intervalAdjustComboHandled) {
    stopAllMotors();
    return true;
  }

  return false;
}

// SELECT + PAD_RIGHT increases stepDist.
// SELECT + PAD_LEFT decreases stepDist.
// stepDist changes by 10 ms per press.
// This is only active while no auto mode is running so it does not conflict
// with active timelapse or bounce motion.
bool handleTimelapseStepDistAdjustment() {
  bool adjustmentAllowed = !isAutoModeActive();
  bool stepDistAdjustUpComboRawActive = ps2x.Button(PSB_SELECT) && ps2x.Button(PSB_PAD_RIGHT);
  bool stepDistAdjustDownComboRawActive = ps2x.Button(PSB_SELECT) && ps2x.Button(PSB_PAD_LEFT);

  if (stepDistAdjustUpComboRawActive && !lastStepDistAdjustUpComboActive) {
    if (adjustmentAllowed) {
      adjustStepDist(TIMELAPSE_STEP_DIST_ADJUST_INCREMENT_MS);
    } else {
      startLockoutDeniedRumbleFeedback();
      Serial.println("Timelapse stepDist adjustment blocked: auto mode active.");
    }
  }

  if (stepDistAdjustDownComboRawActive && !lastStepDistAdjustDownComboActive) {
    if (adjustmentAllowed) {
      adjustStepDist(-TIMELAPSE_STEP_DIST_ADJUST_INCREMENT_MS);
    } else {
      startLockoutDeniedRumbleFeedback();
      Serial.println("Timelapse stepDist adjustment blocked: auto mode active.");
    }
  }

  lastStepDistAdjustUpComboActive = stepDistAdjustUpComboRawActive;
  lastStepDistAdjustDownComboActive = stepDistAdjustDownComboRawActive;

  bool stepDistAdjustComboHandled = adjustmentAllowed && (stepDistAdjustUpComboRawActive || stepDistAdjustDownComboRawActive);

  if (stepDistAdjustComboHandled) {
    stopAllMotors();
    return true;
  }

  return false;
}

// START + SELECT + SQUARE toggles controller rumble mute.
// This only changes controller vibration output; serial logs remain enabled.
bool handleRumbleMuteToggle() {
  bool rumbleMuteToggleComboActive = ps2x.Button(PSB_START) && ps2x.Button(PSB_SELECT) && ps2x.Button(PSB_SQUARE);

  if (rumbleMuteToggleComboActive && !lastRumbleMuteToggleComboActive) {
    rumbleMuted = !rumbleMuted;
    suppressNextSelectRelease = true;
    suppressNextStartRelease = true;
    stopIntervalRumbleFeedback();

    Serial.print("Controller rumble ");
    Serial.println(rumbleMuted ? "muted." : "unmuted.");

    if (!rumbleMuted) {
      startRumbleUnmuteFeedback();
    }
  }

  lastRumbleMuteToggleComboActive = rumbleMuteToggleComboActive;
  return rumbleMuteToggleComboActive;
}

// L1 + L2 + CIRCLE replays current interval and stepDist rumble patterns.
// This is read-only; no values change.
bool handleSettingsReplay() {
  bool replayAllowed = !isAutoModeActive();
  bool settingsReplayComboActive = ps2x.Button(PSB_L1) && ps2x.Button(PSB_L2) && ps2x.Button(PSB_CIRCLE);

  if (settingsReplayComboActive && !lastSettingsReplayComboActive) {
    if (replayAllowed) {
      startSettingsReplayRumble();
    } else {
      startLockoutDeniedRumbleFeedback();
      Serial.println("Settings replay blocked: auto mode active.");
    }
  }

  lastSettingsReplayComboActive = settingsReplayComboActive;
  return settingsReplayComboActive;
}

void handleDroneModeToggle() {
  if (!ps2x.ButtonReleased(PSB_R3)) {
    return;
  }

  if (droneMode) {
    exitDroneMode();
  } else {
    enterDroneMode();
  }
}

void logEmergencyStopEvent() {
  Serial.print("EMERGENCY STOP | timelapseMode=");
  Serial.print(timelapseMode);
  Serial.print(" bounce=");
  Serial.print(bounce);
  Serial.print(" intervalSeconds=");
  Serial.print(timelapseIntervalSeconds);
  Serial.print(" stepDistMs=");
  Serial.println(stepDist);
}

// Emergency stop combo: L1 + L2 + R1 + R2
// Immediately stops all motor outputs and clears active auto modes.
bool handleEmergencyStop() {
  bool emergencyStopComboActive = ps2x.Button(PSB_L1) && ps2x.Button(PSB_L2) && ps2x.Button(PSB_R1) && ps2x.Button(PSB_R2);

  if (!emergencyStopComboActive) {
    if (lastEmergencyStopComboActive) {
      Serial.println("EMERGENCY STOP RELEASED | controls re-enabled");
      startEmergencyReleaseRumbleFeedback();
      stopIntervalRumbleFeedback();
    }
    lastEmergencyStopComboActive = false;
    return false;
  }

  if (!lastEmergencyStopComboActive) {
    logEmergencyStopEvent();
    resetTimelapseState();
    resetBounceState();
    if (droneMode) {
      resetFlowlapseSession(false);
      Serial.println("Flowlapse: canceled by emergency stop.");
    }
    stopIntervalRumbleFeedback();
  }

  stopAllMotors();
  lastEmergencyStopComboActive = true;
  return true;
}

const char* getTimelapseModeLabel(int mode) {
  switch (mode) {
    // Mode 1: swing left, boom down
    case 1:
      return "Timelapse Mode 1: Swing left, boom down";
    // Mode 2: swing left, boom up
    case 2:
      return "Timelapse Mode 2: Swing left, boom up";
    // Mode 3: swing right, boom up
    case 3:
      return "Timelapse Mode 3: Swing right, boom up";
    // Mode 4: swing right, boom down
    case 4:
      return "Timelapse Mode 4: Swing right, boom down";
    // Mode 5: swing left
    case 5:
      return "Timelapse Mode 5: Swing left";
    // Mode 6: boom up
    case 6:
      return "Timelapse Mode 6: Boom up";
    // Mode 7: swing right
    case 7:
      return "Timelapse Mode 7: Swing right";
    // Mode 8: boom down
    case 8:
      return "Timelapse Mode 8: Boom down";
    default:
      return nullptr;
  }
}

void applyTimelapseModeOutputs(int mode) {
  switch (mode) {
    // Mode 1: swing left, boom down
    case 1:
      setDirectionalOutput(isSwingReversed, swingLeft, swingRight, HIGH);
      setDirectionalOutput(isPanReversed, panRight, panLeft, HIGH);
      setDirectionalOutput(isLiftReversed, liftDown, liftUp, HIGH);
      setDirectionalOutput(isTiltReversed, tiltUp, tiltDown, HIGH);
      break;
    // Mode 2: swing left, boom up
    case 2:
      setDirectionalOutput(isSwingReversed, swingLeft, swingRight, HIGH);
      setDirectionalOutput(isPanReversed, panRight, panLeft, HIGH);
      setDirectionalOutput(isLiftReversed, liftUp, liftDown, HIGH);
      setDirectionalOutput(isTiltReversed, tiltDown, tiltUp, HIGH);
      break;
    // Mode 3: swing right, boom up
    case 3:
      setDirectionalOutput(isSwingReversed, swingRight, swingLeft, HIGH);
      setDirectionalOutput(isPanReversed, panLeft, panRight, HIGH);
      setDirectionalOutput(isLiftReversed, liftUp, liftDown, HIGH);
      setDirectionalOutput(isTiltReversed, tiltDown, tiltUp, HIGH);
      break;
    // Mode 4: swing right, boom down
    case 4:
      setDirectionalOutput(isSwingReversed, swingRight, swingLeft, HIGH);
      setDirectionalOutput(isPanReversed, panLeft, panRight, HIGH);
      setDirectionalOutput(isLiftReversed, liftDown, liftUp, HIGH);
      setDirectionalOutput(isTiltReversed, tiltUp, tiltDown, HIGH);
      break;
    // Mode 5: swing left
    case 5:
      setDirectionalOutput(isSwingReversed, swingLeft, swingRight, HIGH);
      setDirectionalOutput(isPanReversed, panRight, panLeft, HIGH);
      break;
    // Mode 6: boom up
    case 6:
      setDirectionalOutput(isLiftReversed, liftUp, liftDown, HIGH);
      setDirectionalOutput(isTiltReversed, tiltDown, tiltUp, HIGH);
      break;
    // Mode 7: swing right
    case 7:
      setDirectionalOutput(isSwingReversed, swingRight, swingLeft, HIGH);
      setDirectionalOutput(isPanReversed, panLeft, panRight, HIGH);
      break;
    // Mode 8: boom down
    case 8:
      setDirectionalOutput(isLiftReversed, liftDown, liftUp, HIGH);
      setDirectionalOutput(isTiltReversed, tiltUp, tiltDown, HIGH);
      break;
  }
}

void resetTimelapseState() {
  timelapseMode = 0;
  timelapsePhase = TIMELAPSE_PHASE_IDLE;
  timelapsePhaseStartMs = 0;
  digitalWrite(trigger, HIGH);
  stopAllMotors();
}

void resetBounceState() {
  bounce = 0;
  stage = 0;
  bouncePhaseStartMs = 0;
  bounceMoveDurationMs = 0;
  stopAllMotors();
}

// Returns 1-8 based on left-stick position, or 0 if no match.
// Used by both timelapse and bounce mode selection.
int stickPositionToMode(int stickX, int stickY) {
  if (stickX < STICK_MODE_LOW_THRESHOLD  && stickY > STICK_MODE_HIGH_THRESHOLD) return 1;
  if (stickX < STICK_MODE_LOW_THRESHOLD  && stickY < STICK_MODE_LOW_THRESHOLD)  return 2;
  if (stickX > STICK_MODE_HIGH_THRESHOLD && stickY < STICK_MODE_LOW_THRESHOLD)  return 3;
  if (stickX > STICK_MODE_HIGH_THRESHOLD && stickY > STICK_MODE_HIGH_THRESHOLD) return 4;
  if (stickX == STICK_MIN) return 5;
  if (stickY == STICK_MIN) return 6;
  if (stickX == STICK_MAX) return 7;
  if (stickY == STICK_MAX) return 8;
  return 0;
}

void updateTimelapseModeSelection() {
  if (suppressNextSelectRelease) {
    if (ps2x.ButtonReleased(PSB_SELECT)) {
      suppressNextSelectRelease = false;
    }
    return;
  }

  if (timelapseMode != 0 || !ps2x.ButtonReleased(PSB_SELECT)) {
    return;
  }
  timelapseMode = stickPositionToMode(leftStickXvalue, leftStickYvalue);
  if (timelapseMode == 0) {
    Serial.println("Timelapse: stick near center at SELECT release, no mode started.");
    return;
  }
  const char* tlLabel = getTimelapseModeLabel(timelapseMode);
  if (tlLabel != nullptr) {
    Serial.println(tlLabel);
  }
}

const char* getBounceModeSerialLabel(int mode) {
  switch (mode) {
    case 1: return "Bounce Mode 1: Swing left, boom down";
    case 2: return "Bounce Mode 2: Swing left, boom up";
    case 3: return "Bounce Mode 3: Swing right, boom up";
    case 4: return "Bounce Mode 4: Swing right, boom down";
    case 5: return "Bounce Mode 5: Swing left";
    case 6: return "Bounce Mode 6: Boom up";
    case 7: return "Bounce Mode 7: Swing right";
    case 8: return "Bounce Mode 8: Boom down";
    default: return nullptr;
  }
}

void updateBounceModeSelection() {
  if (suppressNextStartRelease) {
    if (ps2x.ButtonReleased(PSB_START)) {
      suppressNextStartRelease = false;
    }
    return;
  }

  if (bounce != 0 || !ps2x.ButtonReleased(PSB_START)) {
    return;
  }
  bounce = stickPositionToMode(leftStickXvalue, leftStickYvalue);
  if (bounce == 0) {
    Serial.println("Bounce: stick near center at START release, no mode started.");
    return;
  }
  const char* bounceLabel = getBounceModeSerialLabel(bounce);
  if (bounceLabel != nullptr) {
    Serial.println(bounceLabel);
  }
}

// Drives the motors for the given bounce mode toward or away from the endpoint.
// towardEndpoint=true  → moving toward the target position (stage 0 travel + stage 1 second half)
// towardEndpoint=false → moving back toward the start position (stage 1 first half)
// state=HIGH turns the motor on; state=LOW turns it off.
// DIP-switch reversal is handled inside setDirectionalOutput().
void setBounceModeOutputs(int mode, bool towardEndpoint, uint8_t state) {
  switch (mode) {
    case 1: // swing left + boom down
      if (towardEndpoint) {
        setDirectionalOutput(isSwingReversed, swingLeft,  swingRight, state);
        setDirectionalOutput(isPanReversed,   panRight,   panLeft,    state);
        setDirectionalOutput(isLiftReversed,  liftDown,   liftUp,     state);
        setDirectionalOutput(isTiltReversed,  tiltUp,     tiltDown,   state);
      } else {
        setDirectionalOutput(isSwingReversed, swingRight, swingLeft,  state);
        setDirectionalOutput(isPanReversed,   panLeft,    panRight,   state);
        setDirectionalOutput(isLiftReversed,  liftUp,     liftDown,   state);
        setDirectionalOutput(isTiltReversed,  tiltDown,   tiltUp,     state);
      }
      break;
    case 2: // swing left + boom up
      if (towardEndpoint) {
        setDirectionalOutput(isSwingReversed, swingLeft,  swingRight, state);
        setDirectionalOutput(isPanReversed,   panRight,   panLeft,    state);
        setDirectionalOutput(isLiftReversed,  liftUp,     liftDown,   state);
        setDirectionalOutput(isTiltReversed,  tiltDown,   tiltUp,     state);
      } else {
        setDirectionalOutput(isSwingReversed, swingRight, swingLeft,  state);
        setDirectionalOutput(isPanReversed,   panLeft,    panRight,   state);
        setDirectionalOutput(isLiftReversed,  liftDown,   liftUp,     state);
        setDirectionalOutput(isTiltReversed,  tiltUp,     tiltDown,   state);
      }
      break;
    case 3: // swing right + boom up
      if (towardEndpoint) {
        setDirectionalOutput(isSwingReversed, swingRight, swingLeft,  state);
        setDirectionalOutput(isPanReversed,   panLeft,    panRight,   state);
        setDirectionalOutput(isLiftReversed,  liftUp,     liftDown,   state);
        setDirectionalOutput(isTiltReversed,  tiltDown,   tiltUp,     state);
      } else {
        setDirectionalOutput(isSwingReversed, swingLeft,  swingRight, state);
        setDirectionalOutput(isPanReversed,   panRight,   panLeft,    state);
        setDirectionalOutput(isLiftReversed,  liftDown,   liftUp,     state);
        setDirectionalOutput(isTiltReversed,  tiltUp,     tiltDown,   state);
      }
      break;
    case 4: // swing right + boom down
      if (towardEndpoint) {
        setDirectionalOutput(isSwingReversed, swingRight, swingLeft,  state);
        setDirectionalOutput(isPanReversed,   panLeft,    panRight,   state);
        setDirectionalOutput(isLiftReversed,  liftDown,   liftUp,     state);
        setDirectionalOutput(isTiltReversed,  tiltUp,     tiltDown,   state);
      } else {
        setDirectionalOutput(isSwingReversed, swingLeft,  swingRight, state);
        setDirectionalOutput(isPanReversed,   panRight,   panLeft,    state);
        setDirectionalOutput(isLiftReversed,  liftUp,     liftDown,   state);
        setDirectionalOutput(isTiltReversed,  tiltDown,   tiltUp,     state);
      }
      break;
    case 5: // swing left only
      if (towardEndpoint) {
        setDirectionalOutput(isSwingReversed, swingLeft,  swingRight, state);
        setDirectionalOutput(isPanReversed,   panRight,   panLeft,    state);
      } else {
        setDirectionalOutput(isSwingReversed, swingRight, swingLeft,  state);
        setDirectionalOutput(isPanReversed,   panLeft,    panRight,   state);
      }
      break;
    case 6: // boom up only
      if (towardEndpoint) {
        setDirectionalOutput(isLiftReversed,  liftUp,     liftDown,   state);
        setDirectionalOutput(isTiltReversed,  tiltDown,   tiltUp,     state);
      } else {
        setDirectionalOutput(isLiftReversed,  liftDown,   liftUp,     state);
        setDirectionalOutput(isTiltReversed,  tiltUp,     tiltDown,   state);
      }
      break;
    case 7: // swing right only
      if (towardEndpoint) {
        setDirectionalOutput(isSwingReversed, swingRight, swingLeft,  state);
        setDirectionalOutput(isPanReversed,   panLeft,    panRight,   state);
      } else {
        setDirectionalOutput(isSwingReversed, swingLeft,  swingRight, state);
        setDirectionalOutput(isPanReversed,   panRight,   panLeft,    state);
      }
      break;
    case 8: // boom down only
      if (towardEndpoint) {
        setDirectionalOutput(isLiftReversed,  liftDown,   liftUp,     state);
        setDirectionalOutput(isTiltReversed,  tiltUp,     tiltDown,   state);
      } else {
        setDirectionalOutput(isLiftReversed,  liftUp,     liftDown,   state);
        setDirectionalOutput(isTiltReversed,  tiltDown,   tiltUp,     state);
      }
      break;
  }
}

// Stops only the motors used by the given bounce mode.
// Called at the turnaround point between stage 0 and stage 1,
// and during direction switches inside stage 1.
// Modes 1-4 use all four axes; modes 5/7 use swing only; modes 6/8 use lift only.
void stopBounceModeOutputs(int mode) {
  switch (mode) {
    case 1: // swing left + boom down
    case 2: // swing left + boom up
    case 3: // swing right + boom up
    case 4: // swing right + boom down
      digitalWrite(swingLeft, LOW);
      digitalWrite(swingRight, LOW);
      digitalWrite(panLeft, LOW);
      digitalWrite(panRight, LOW);
      digitalWrite(liftUp, LOW);
      digitalWrite(liftDown, LOW);
      digitalWrite(tiltUp, LOW);
      digitalWrite(tiltDown, LOW);
      break;
    case 5: // swing left only
    case 7: // swing right only
      digitalWrite(swingLeft, LOW);
      digitalWrite(swingRight, LOW);
      digitalWrite(panLeft, LOW);
      digitalWrite(panRight, LOW);
      break;
    case 6: // boom up only
    case 8: // boom down only
      digitalWrite(liftUp, LOW);
      digitalWrite(liftDown, LOW);
      digitalWrite(tiltUp, LOW);
      digitalWrite(tiltDown, LOW);
      break;
  }
}

void advanceBounceToStage1() {
  unsigned long now = millis();
  unsigned long measuredMoveDurationMs = now - bouncePhaseStartMs;

  if (measuredMoveDurationMs < BOUNCE_MIN_MOVE_DURATION_MS) {
    startLockoutDeniedRumbleFeedback();
    Serial.print("Bounce endpoint rejected: minimum move time is ");
    Serial.print(BOUNCE_MIN_MOVE_DURATION_MS);
    Serial.println("ms.");
    return;
  }

  stopBounceModeOutputs(bounce);
  bounceMoveDurationMs = measuredMoveDurationMs;
  bouncePhaseStartMs = now;
  stage = 1;
  startL3EndpointRumbleFeedback();
  Serial.print("Bounce endpoint set: ");
  Serial.print(bounceMoveDurationMs);
  Serial.println("ms travel time");
}

void handleBounceStage0(unsigned long now) {
  if (bounce == 0 || stage != 0) {
    return;
  }

  if (bouncePhaseStartMs == 0) {
    bouncePhaseStartMs = now;
  }

  setBounceModeOutputs(bounce, true, HIGH);

  if (ps2x.ButtonReleased(PSB_L3)) {
    advanceBounceToStage1();
  }
}

void handleBounceStage1(unsigned long now) {
  if (bounce == 0 || stage != 1) {
    return;
  }

  if (bounceMoveDurationMs == 0) {
    return;
  }

  unsigned long elapsed = (now - bouncePhaseStartMs) % (bounceMoveDurationMs * 2);

  if (elapsed < bounceMoveDurationMs) {
    // first half: traveling away from endpoint (back toward start)
    setBounceModeOutputs(bounce, true, LOW);
    setBounceModeOutputs(bounce, false, HIGH);
  } else {
    // second half: traveling toward endpoint
    setBounceModeOutputs(bounce, false, LOW);
    setBounceModeOutputs(bounce, true, HIGH);
  }
}

void handleActiveTimelapseMode(unsigned long now) {
  if (timelapseMode == 0) {
    return;
  }

  const char* modeLabel = getTimelapseModeLabel(timelapseMode);
  if (modeLabel == nullptr) {
    resetTimelapseState();
    return;
  }

  switch (timelapsePhase) {
    case TIMELAPSE_PHASE_IDLE:
      digitalWrite(trigger, LOW);
      timelapsePhase = TIMELAPSE_PHASE_TRIGGER_LOW;
      timelapsePhaseStartMs = now;
      break;
    case TIMELAPSE_PHASE_TRIGGER_LOW:
      if (now - timelapsePhaseStartMs >= static_cast<unsigned long>(timelapseIntervalMs / 2)) {
        digitalWrite(trigger, HIGH);
        timelapsePhase = TIMELAPSE_PHASE_TRIGGER_HIGH;
        timelapsePhaseStartMs = now;
      }
      break;
    case TIMELAPSE_PHASE_TRIGGER_HIGH:
      if (now - timelapsePhaseStartMs >= static_cast<unsigned long>(timelapseIntervalMs / 2)) {
        applyTimelapseModeOutputs(timelapseMode);
        timelapsePhase = TIMELAPSE_PHASE_MOVE_ACTIVE;
        timelapsePhaseStartMs = now;
      }
      break;
    case TIMELAPSE_PHASE_MOVE_ACTIVE:
      if (now - timelapsePhaseStartMs >= static_cast<unsigned long>(stepDist)) {
        stopAllMotors();
        timelapsePhase = TIMELAPSE_PHASE_IDLE;
        timelapsePhaseStartMs = now;
      }
      break;
    default:
      resetTimelapseState();
      return;
  }
}

void printDroneTuningProfile() {
  Serial.print("Drone tuning | expo S/L/P/T=");
  Serial.print(DRONE_SWING_EXPO_PERCENT);
  Serial.print("/");
  Serial.print(DRONE_LIFT_EXPO_PERCENT);
  Serial.print("/");
  Serial.print(DRONE_PAN_EXPO_PERCENT);
  Serial.print("/");
  Serial.println(DRONE_TILT_EXPO_PERCENT);

  Serial.print("Drone tuning | deadband S/L/P/T=");
  Serial.print(DRONE_SWING_DEADBAND);
  Serial.print("/");
  Serial.print(DRONE_LIFT_DEADBAND);
  Serial.print("/");
  Serial.print(DRONE_PAN_DEADBAND);
  Serial.print("/");
  Serial.println(DRONE_TILT_DEADBAND);

  Serial.print("Drone tuning | max tier S/L/P/T=");
  Serial.print(DRONE_SWING_MAX_SPEED_TIER);
  Serial.print("/");
  Serial.print(DRONE_LIFT_MAX_SPEED_TIER);
  Serial.print("/");
  Serial.print(DRONE_PAN_MAX_SPEED_TIER);
  Serial.print("/");
  Serial.println(DRONE_TILT_MAX_SPEED_TIER);

  Serial.print("Drone tuning | modifier precision/boost enabled=");
  Serial.print(DRONE_ENABLE_PRECISION_MODIFIER ? "Y" : "N");
  Serial.print("/");
  Serial.println(DRONE_ENABLE_BOOST_MODIFIER ? "Y" : "N");

  Serial.print("Drone tuning | L2+R2 neutral mode=");
  Serial.println(DRONE_L2_R2_NEUTRAL_MODE ? "Y" : "N");

  Serial.print("Drone tuning | tier thresholds med/high=");
  Serial.print(DRONE_SPEED_TIER_MED_THRESHOLD);
  Serial.print("/");
  Serial.println(DRONE_SPEED_TIER_HIGH_THRESHOLD);

  Serial.print("Drone tuning | idle timeout ms=");
  if (DRONE_IDLE_TIMEOUT_MS == 0) {
    Serial.println("disabled");
  } else {
    Serial.println(DRONE_IDLE_TIMEOUT_MS);
  }

  Serial.print("Drone tuning | serial log enabled=");
  Serial.println(DRONE_SERIAL_LOG_ENABLED ? "Y" : "N");
}

void setup() {

  Serial.begin(9600);
  updateIntervalMs();

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

  resetTimelapseState();

  delay(CONTROLLER_STARTUP_DELAY_MS);
  configureController();
  detectControllerType();
  Serial.println("START + PAD_UP/DOWN adjusts timelapseIntervalSeconds.");
  Serial.println("SELECT + PAD_RIGHT/LEFT adjusts stepDist by 10 ms.");
  Serial.println("START + SELECT + SQUARE toggles controller rumble mute.");
  Serial.println("L1 + L2 + CIRCLE replays current settings as rumble patterns.");
  Serial.print("Boot settings: interval=");
  Serial.print(timelapseIntervalSeconds);
  Serial.print("s, stepDist=");
  Serial.print(stepDist);
  Serial.println("ms");
  printDroneTuningProfile();
}

void loop() {

  unsigned long now = millis();

  if (error != 0) {
    if (now - lastControllerRetryMs >= CONTROLLER_RETRY_INTERVAL_MS) {
      lastControllerRetryMs = now;
      Serial.println("Controller init failed. Retrying config...");
      configureController();
      if (error == 0) {
        detectControllerType();
      }
    }
    return;
  }

  if (controllerType == 2) {
    return;
  }

  const bool isDualShockType = (controllerType == 1 || controllerType == 3);
  if (!isDualShockType) {
    if (!unsupportedControllerWarningShown) {
      Serial.print("Unsupported controller type: ");
      Serial.println(controllerType);
      unsupportedControllerWarningShown = true;
    }
    return;
  }

  unsupportedControllerWarningShown = false;

  handleIntervalRumbleFeedback(now);
  ps2x.read_gamepad(false, rumbleMuted ? 0 : vibrate);

  isSwingReversed = (digitalRead(DIP_SWITCH_1) == HIGH);
  isPanReversed = (digitalRead(DIP_SWITCH_2) == HIGH);
  isLiftReversed = (digitalRead(DIP_SWITCH_3) == HIGH);
  isTiltReversed = (digitalRead(DIP_SWITCH_4) == HIGH);
  isFocusReversed = (digitalRead(DIP_SWITCH_5) == HIGH);

  rightStickYvalue = ps2x.Analog(PSS_RY);
  rightStickXvalue = ps2x.Analog(PSS_RX);
  leftStickYvalue = ps2x.Analog(PSS_LY);
  leftStickXvalue = ps2x.Analog(PSS_LX);

  if (handleEmergencyStop()) {
    return;
  }

  handleDroneModeToggle();

  if (!droneMode) {
    if (handleTimelapseIntervalAdjustment()) {
      return;
    }

    if (handleTimelapseStepDistAdjustment()) {
      return;
    }

    if (handleRumbleMuteToggle()) {
      return;
    }

    if (handleSettingsReplay()) {
      return;
    }
  }

  if (droneMode) {
    if (DRONE_IDLE_TIMEOUT_MS > 0 && millis() - droneLastActivityMs >= DRONE_IDLE_TIMEOUT_MS) {
      Serial.println("Drone idle timeout - auto-exiting drone mode");
      exitDroneMode();
      return;
    }

    float flowlapseDeltaSeconds = getFlowlapseDeltaSeconds(now);
    handleDroneFlowlapseWorkflow(now, flowlapseDeltaSeconds);
    return;
  }

  handleAxisSpeedControl(PSB_R1, liftSpeedUp, tiltSpeedUp);
  handleAxisSpeedControl(PSB_R2, liftSpeedDown, tiltSpeedDown);
  handleAxisSpeedControl(PSB_L1, panSpeedUp, swingSpeedUp);
  handleAxisSpeedControl(PSB_L2, panSpeedDown, swingSpeedDown);

  handleSwingOnly(PSB_PAD_LEFT, swingLeft, swingRight);
  handleSwingOnly(PSB_PAD_RIGHT, swingRight, swingLeft);
  handleSwingAndPan(PSB_PAD_LEFT, swingLeft, swingRight, panRight, panLeft);
  handleSwingAndPan(PSB_PAD_RIGHT, swingRight, swingLeft, panLeft, panRight);

  handlePanTrimAxis();
  handlePanAxis();

  handleLiftOnly(PSB_PAD_UP, liftUp, liftDown);
  handleLiftOnly(PSB_PAD_DOWN, liftDown, liftUp);
  handleLiftAndTilt(PSB_PAD_UP, liftUp, liftDown, tiltDown, tiltUp);
  handleLiftAndTilt(PSB_PAD_DOWN, liftDown, liftUp, tiltUp, tiltDown);

  handleTiltTrimAxis();
  handleTiltAxis();
  handleFocusAxis();

  updateTimelapseModeSelection();
  handleActiveTimelapseMode(now);

  updateBounceModeSelection();
  handleBounceStage0(now);
  handleBounceStage1(now);
}

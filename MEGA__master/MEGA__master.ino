#include <EEPROM.h>
#include <PS2X_lib.h>  //for v1.6
#include <stdio.h>
#include <string.h>

constexpr uint8_t PS2_DAT = 10;
constexpr uint8_t PS2_CMD = 9;
constexpr uint8_t PS2_SEL = 8;
constexpr uint8_t PS2_CLK = 11;

constexpr bool PRESSURES = false;
constexpr bool RUMBLE = true;
constexpr bool DEBUG_EDGE_EVENTS = false;
constexpr bool DEBUG_DRONE_TIERS = true; // Log speed tier transitions in real-time

PS2X ps2x;
int error = 0;
byte controllerType = 0;
byte vibrate = 0;

// Output pins and control state
const uint8_t swingLeft = 24;
const uint8_t swingRight = 25;
const uint8_t swingSpeedUp = 26;
const uint8_t swingSpeedDown = 27;
uint8_t swingSoloMode = 0;

const uint8_t panLeft = 46;
const uint8_t panRight = 48;
const uint8_t panSpeedUp = 50;
const uint8_t panSpeedDown = 52;
uint8_t panStop = 0;

const uint8_t panSpeedUpOnly = 29; //lower
const uint8_t panSpeedDownOnly = 45; //higher

const uint8_t liftDown = 30;
const uint8_t liftUp = 31;
const uint8_t liftSpeedUp = 32;
const uint8_t liftSpeedDown = 33;
uint8_t liftSoloMode = 0;

const uint8_t tiltSpeedUpOnly = 42; //lower val
const uint8_t tiltSpeedDownOnly = 44; //higher val

const uint8_t tiltDown = 34;
const uint8_t tiltUp = 36;
const uint8_t tiltSpeedUp = 38;
const uint8_t tiltSpeedDown = 40;
uint8_t tiltStop = 0;

const uint8_t focusLeft = 47;
const uint8_t focusRight = 49;
const uint8_t focusSpeedUp = 51;
const uint8_t focusSpeedDown = 53;

uint8_t motionFlags = 0;
uint8_t droneAxisLastActiveFlags = 0;
bool lastSwingSoloLeftActive = false;
bool lastSwingSoloRightActive = false;
bool lastLiftSoloUpActive = false;
bool lastLiftSoloDownActive = false;
int8_t lastPanSoloDirection = 0;
int8_t lastTiltSoloDirection = 0;

int leftStickXvalue;
int leftStickYvalue;
int rightStickXvalue;
int rightStickYvalue;

// Timelapse Variables
uint8_t timelapseMode = 0;
constexpr uint8_t DEFAULT_TIMELAPSE_INTERVAL_SECONDS = 15;
uint8_t timelapseIntervalSeconds = DEFAULT_TIMELAPSE_INTERVAL_SECONDS;
unsigned long timelapseIntervalMs;
constexpr int DEFAULT_TIMELAPSE_STEP_DIST_MS = 250;  // Increased from 100ms for smoother motion
int stepDist = DEFAULT_TIMELAPSE_STEP_DIST_MS;
constexpr unsigned long DEFAULT_TIMELAPSE_SETTLE_DWELL_MS = 500;
unsigned long timelapseSettleDwellMs = DEFAULT_TIMELAPSE_SETTLE_DWELL_MS;
constexpr uint8_t DEFAULT_TIMELAPSE_MAX_SPEED_STAGE = 4;  // 1-4; limits max velocity during ramp
constexpr uint8_t TIMELAPSE_MAX_SPEED_STAGE_MIN = 1;
constexpr uint8_t TIMELAPSE_MAX_SPEED_STAGE_MAX = 4;
uint8_t timelapseMaxSpeedStage = DEFAULT_TIMELAPSE_MAX_SPEED_STAGE;
const uint8_t trigger = 28;

// Timelapse motion ramping parameters for smooth camera movement
constexpr bool DEFAULT_TIMELAPSE_ANTI_BACKLASH_ENABLED = true;
constexpr uint8_t TIMELAPSE_ANTI_BACKLASH_DURATION_MS = 60;
bool timelapseAntiBacklashEnabled = DEFAULT_TIMELAPSE_ANTI_BACKLASH_ENABLED;
constexpr uint8_t TIMELAPSE_RAMP_UP_DURATION_MS = 40;      // Acceleration phase duration
constexpr uint8_t TIMELAPSE_RAMP_DOWN_DURATION_MS = 40;    // Deceleration phase duration
constexpr uint8_t TIMELAPSE_CRUISE_MIN_DURATION_MS = 100;  // Minimum cruise time at full speed
uint8_t timelapseCurrentSpeedStage = 0;                    // Track current speed stage during motion

enum TimelapsePhase : uint8_t {
  TIMELAPSE_PHASE_IDLE,
  TIMELAPSE_PHASE_TRIGGER_LOW,
  TIMELAPSE_PHASE_TRIGGER_HIGH,
  TIMELAPSE_PHASE_MOVE_PRELOAD,
  TIMELAPSE_PHASE_MOVE_RAMP_UP,
  TIMELAPSE_PHASE_MOVE_CRUISE,
  TIMELAPSE_PHASE_MOVE_RAMP_DOWN,
  TIMELAPSE_PHASE_SETTLE
};

TimelapsePhase timelapsePhase = TIMELAPSE_PHASE_IDLE;
unsigned long timelapsePhaseStartMs = 0;

enum IntervalRumblePhase : uint8_t {
  INTERVAL_RUMBLE_IDLE,
  INTERVAL_RUMBLE_LONG_ACTIVE,
  INTERVAL_RUMBLE_LONG_PAUSE,
  INTERVAL_RUMBLE_SHORT_ACTIVE,
  INTERVAL_RUMBLE_SHORT_PAUSE
};

enum StepDistRumblePhase : uint8_t {
  STEP_DIST_RUMBLE_IDLE,
  STEP_DIST_RUMBLE_LONG_ACTIVE,
  STEP_DIST_RUMBLE_LONG_PAUSE
};

enum PendingRumblePattern : uint8_t {
  PENDING_RUMBLE_NONE,
  PENDING_RUMBLE_INTERVAL,
  PENDING_RUMBLE_STEP_DIST
};

enum FeedbackRumblePhase : uint8_t {
  FEEDBACK_RUMBLE_IDLE,
  FEEDBACK_RUMBLE_ON,
  FEEDBACK_RUMBLE_PAUSE
};

enum FrameCountCompleteRumblePhase : uint8_t {
  FRAMECOUNT_COMPLETE_RUMBLE_IDLE,
  FRAMECOUNT_COMPLETE_RUMBLE_LONG_ON,
  FRAMECOUNT_COMPLETE_RUMBLE_LONG_PAUSE,
  FRAMECOUNT_COMPLETE_RUMBLE_SHORT_ON,
  FRAMECOUNT_COMPLETE_RUMBLE_SHORT_PAUSE
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
FrameCountCompleteRumblePhase frameCountCompleteRumblePhase = FRAMECOUNT_COMPLETE_RUMBLE_IDLE;
unsigned long frameCountCompleteRumblePhaseStartMs = 0;

// Motion Control Variables
uint8_t bounce = 0;
uint8_t stage = 0;
unsigned long bouncePhaseStartMs = 0;
unsigned long bounceMoveDurationMs = 0;
uint8_t bounceSwingTier = 1;
uint8_t bouncePanTier = 1;
uint8_t bounceLiftTier = 1;
uint8_t bounceTiltTier = 1;

enum StatusModeKind : uint8_t {
  STATUS_MODE_MANUAL = 0,
  STATUS_MODE_DRONE,
  STATUS_MODE_TIMELAPSE,
  STATUS_MODE_BOUNCE
};

StatusModeKind lastBroadcastModeKind = STATUS_MODE_MANUAL;
uint8_t lastBroadcastModeVariant = 0;
bool modeStatusInitialized = false;

constexpr uint8_t DIP_SWITCH_1 = 35;
constexpr uint8_t DIP_SWITCH_2 = 43;
constexpr uint8_t DIP_SWITCH_3 = 37;
constexpr uint8_t DIP_SWITCH_4 = 39;
constexpr uint8_t DIP_SWITCH_5 = 41;
constexpr int EEPROM_SETTINGS_ADDRESS = 0;
constexpr uint16_t PERSISTED_SETTINGS_MAGIC = 0x4D52;
constexpr uint8_t PERSISTED_SETTINGS_VERSION = 5;
constexpr uint8_t PERSISTED_SETTINGS_FLAG_RUMBLE_MUTED = 0x01;
constexpr uint8_t PERSISTED_SETTINGS_FLAG_HOME_VALID = 0x02;
constexpr unsigned long CONTROLLER_STARTUP_DELAY_MS = 300;
constexpr unsigned long CONTROLLER_RETRY_INTERVAL_MS = 2000;
constexpr unsigned long CONTROLLER_OK_BROADCAST_INTERVAL_MS = 1500;
constexpr uint8_t DEFAULT_AXIS_SPEED_STAGE = 1; // 0=slowest, 4=fastest — set low so L1/R1 have room to go up
constexpr unsigned long SPEED_STAGE_PULSE_HIGH_MS = 75;
constexpr unsigned long SPEED_STAGE_PULSE_LOW_MS = 75;
constexpr bool WILL_TEST_SPEED_BUILD = true;
constexpr bool WILL_TEST_BYPASS_LOGS = false;
constexpr bool WILL_TEST_LEGACY_HELD_SPEED_SIGNALS = false;
constexpr unsigned long WILL_TEST_LED_PULSE_MS = 18;
constexpr int STICK_MIN = 0;
constexpr int STICK_CENTER = 128;
constexpr int STICK_MAX = 255;
constexpr int STICK_DEADBAND = 5;   // tolerance for analog stick exact-equality comparisons
constexpr int STICK_MODE_LOW_THRESHOLD = 123;
constexpr int STICK_MODE_HIGH_THRESHOLD = 133;
constexpr uint8_t PAN_STOP_NONE = 0;
constexpr uint8_t PAN_STOP_ACTIVE = 1;
constexpr uint8_t PAN_STOP_TRIM = 2;
constexpr uint8_t TILT_STOP_NONE = 0;
constexpr uint8_t TILT_STOP_ACTIVE = 1;
constexpr uint8_t TILT_STOP_TRIM = 2;
constexpr int TIMELAPSE_INTERVAL_MIN_SECONDS = 1;
constexpr int TIMELAPSE_INTERVAL_MAX_SECONDS = 99;
constexpr int TIMELAPSE_STEP_DIST_MIN_MS = 20;
constexpr int TIMELAPSE_STEP_DIST_MAX_MS = 150;
constexpr int TIMELAPSE_STEP_DIST_ADJUST_INCREMENT_MS = 10;
constexpr unsigned long TIMELAPSE_SETTLE_DWELL_MIN_MS = 0;
constexpr unsigned long TIMELAPSE_SETTLE_DWELL_MAX_MS = 3000;
constexpr unsigned long TIMELAPSE_SETTLE_DWELL_ADJUST_INCREMENT_MS = 100;
constexpr unsigned long TIMELAPSE_SETTLE_DWELL_MS = 500;
constexpr unsigned long PERSISTED_SETTINGS_RESET_HOLD_MS = 1500;
constexpr unsigned long SETTINGS_TOGGLE_HOLD_MS = 800;
constexpr uint8_t RUMBLE_ACTIVE_INTENSITY = 255;
constexpr unsigned long INTERVAL_RUMBLE_LONG_MS = 1600;
constexpr unsigned long INTERVAL_RUMBLE_LONG_PAUSE_MS = 400;
constexpr unsigned long INTERVAL_RUMBLE_GROUP_SEPARATOR_MS = 2000;
constexpr unsigned long INTERVAL_RUMBLE_SHORT_ON_MS = 600;
constexpr unsigned long INTERVAL_RUMBLE_SHORT_TOTAL_MS = 1200;
constexpr unsigned long STEP_DIST_RUMBLE_LONG_ON_MS = 400;
constexpr unsigned long STEP_DIST_RUMBLE_LONG_TOTAL_MS = 800;
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
constexpr unsigned long FRAMECOUNT_COMPLETE_RUMBLE_LONG_ON_MS = 1000;
constexpr unsigned long FRAMECOUNT_COMPLETE_RUMBLE_LONG_TOTAL_MS = 1200;
constexpr unsigned long FRAMECOUNT_COMPLETE_RUMBLE_SHORT_ON_MS = 450;
constexpr unsigned long FRAMECOUNT_COMPLETE_RUMBLE_SHORT_TOTAL_MS = 600;
constexpr unsigned long BOUNCE_MIN_MOVE_DURATION_MS = 150;
constexpr unsigned long BOUNCE_ENDPOINT_STOP_BAND_MS = 30;
constexpr unsigned long BOUNCE_SPEED_RAMP_WINDOW_MIN_MS = 120;
constexpr unsigned long BOUNCE_SPEED_RAMP_WINDOW_MAX_MS = 600;

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
constexpr uint8_t DRONE_SPEED_TIER_MAX = 4;
constexpr uint8_t DRONE_BOOST_TIER_DELTA = 2;
constexpr int DRONE_SPEED_TIER_MED_THRESHOLD = 35;
constexpr int DRONE_SPEED_TIER_HIGH_THRESHOLD = 55;
constexpr int DRONE_STICK_MAX_DEFLECTION = 127;
constexpr uint8_t DRONE_EXPO_PERCENT = 28;
constexpr uint8_t DRONE_SWING_EXPO_PERCENT = DRONE_EXPO_PERCENT;
constexpr uint8_t DRONE_LIFT_EXPO_PERCENT = DRONE_EXPO_PERCENT;
constexpr uint8_t DRONE_PAN_EXPO_PERCENT = DRONE_EXPO_PERCENT;
constexpr uint8_t DRONE_TILT_EXPO_PERCENT = DRONE_EXPO_PERCENT;
constexpr int DRONE_SWING_DEADBAND = 6;
constexpr int DRONE_LIFT_DEADBAND = 6;
constexpr int DRONE_PAN_DEADBAND = 6;
constexpr int DRONE_TILT_DEADBAND = 6;
constexpr int DRONE_UI_DEADBAND = 4;
constexpr uint8_t DRONE_SWING_MAX_SPEED_TIER = DRONE_SPEED_TIER_MAX;
constexpr uint8_t DRONE_LIFT_MAX_SPEED_TIER = DRONE_SPEED_TIER_MAX;
constexpr uint8_t DRONE_PAN_MAX_SPEED_TIER = DRONE_SPEED_TIER_MAX;
constexpr uint8_t DRONE_TILT_MAX_SPEED_TIER = DRONE_SPEED_TIER_MAX;
constexpr bool DRONE_ENABLE_PRECISION_MODIFIER = true;
constexpr bool DRONE_ENABLE_BOOST_MODIFIER = true;
constexpr uint8_t DRONE_FIXED_STICK_SPEED_TIER = DRONE_SPEED_TIER_MED;
constexpr bool DRONE_L2_PRIORITY_OVER_BOOST = true;
constexpr float DRONE_MICRO_MOTION_SPEED_RATIO = 0.12f; // L2 precision-mode duty scale
constexpr unsigned long DRONE_MANUAL_TIER_STEP_INTERVAL_MS = 120;
constexpr unsigned long DRONE_SPEED_STAGE_PULSE_HIGH_MS = 8;
constexpr unsigned long DRONE_SPEED_STAGE_PULSE_LOW_MS = 8;
constexpr unsigned long DRONE_STICK_SPEED_TOGGLE_RUMBLE_ON_MS = 170;
constexpr unsigned long DRONE_STICK_SPEED_TOGGLE_RUMBLE_TOTAL_MS = 320;
constexpr uint8_t DRONE_STICK_SPEED_TOGGLE_PROPORTIONAL_PULSES = 2;
constexpr uint8_t DRONE_STICK_SPEED_TOGGLE_FIXED_PULSES = 1;
constexpr unsigned long DRONE_IDLE_TIMEOUT_MS = 0UL; // disabled; exit drone mode with R3
constexpr bool DRONE_SERIAL_LOG_ENABLED = true; // set false to silence runtime drone logs
constexpr unsigned long DRONE_UI_BROADCAST_INTERVAL_MS = 10;
constexpr unsigned long DRONE_MODE_EXIT_HOLD_MS = 450;
constexpr unsigned long DRONE_MANUAL_SHUTTER_PULSE_MS = 120;

constexpr uint8_t FLOWLAPSE_MAX_WAYPOINTS = 8;
constexpr bool FLOWLAPSE_LOOP_CAPTURE = false; // if true, capture auto-restarts after completing
constexpr unsigned long FLOWLAPSE_WAYPOINT_RUMBLE_ON_MS = 90;
constexpr unsigned long FLOWLAPSE_WAYPOINT_RUMBLE_TOTAL_MS = 180;
constexpr uint8_t FLOWLAPSE_WAYPOINT_RUMBLE_PULSES = 1;
constexpr uint8_t FLOWLAPSE_WIPE_RUMBLE_PULSES = 3;
constexpr unsigned long FLOWLAPSE_WIPE_RUMBLE_ON_MS = 180;
constexpr unsigned long FLOWLAPSE_WIPE_RUMBLE_TOTAL_MS = 320;
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
constexpr bool FLOWLAPSE_CURVED_PATH_ENABLED = true; // smooth Catmull-Rom path during capture move phase
constexpr unsigned long FLOWLAPSE_WAYPOINT_DWELL_MS = 0UL; // ms to hold still at each waypoint before triggering; 0 = disabled
constexpr unsigned long FLOWLAPSE_DWELL_ADJUST_INCREMENT_MS = 250UL; // step size per SELECT+D-pad press in drone mode
constexpr unsigned long FLOWLAPSE_DWELL_MAX_MS = 5000UL; // upper cap for controller adjustment
constexpr float FLOWLAPSE_PREVIEW_SPEED_SCALE = 0.70f; // 1.0 = same as capture, <1 slower preview
constexpr float FLOWLAPSE_PREVIEW_SPEED_BOOST = 3.0f; // L2 hold multiplier for fast preview (3x normal speed)
constexpr unsigned long FLOWLAPSE_L3_RESET_HOLD_MS = 1500UL; // hold L3 to clear and re-arm recording quickly
constexpr bool FLOWLAPSE_EASE_IN_OUT_ENABLED = true; // smooth segment acceleration/deceleration during capture motion
constexpr float FLOWLAPSE_EASE_STRENGTH = 1.0f; // 0.0 = linear, 1.0 = full easing
constexpr bool FLOWLAPSE_PING_PONG_LOOP = false; // alternate forward/backward segments continuously
constexpr bool FLOWLAPSE_RETURN_TO_FIRST_WAYPOINT_ON_COMPLETE = false; // auto-return to waypoint 1 when non-loop capture finishes
constexpr bool FLOWLAPSE_ARC_LENGTH_SAMPLING_ENABLED = true; // reparameterize curved segments for path-aware spacing
enum FlowlapseArcLengthQualityPreset : uint8_t {
  FLOWLAPSE_ARC_LENGTH_QUALITY_LOW,
  FLOWLAPSE_ARC_LENGTH_QUALITY_MED,
  FLOWLAPSE_ARC_LENGTH_QUALITY_HIGH
};
constexpr FlowlapseArcLengthQualityPreset FLOWLAPSE_ARC_LENGTH_QUALITY_PRESET = FLOWLAPSE_ARC_LENGTH_QUALITY_MED; // low=8, med=16, high=24
constexpr uint8_t FLOWLAPSE_ARC_LENGTH_TABLE_STEPS =
  (FLOWLAPSE_ARC_LENGTH_QUALITY_PRESET == FLOWLAPSE_ARC_LENGTH_QUALITY_LOW) ? 8 :
  (FLOWLAPSE_ARC_LENGTH_QUALITY_PRESET == FLOWLAPSE_ARC_LENGTH_QUALITY_HIGH) ? 24 : 16;
constexpr bool FLOWLAPSE_FRAME_COUNT_MODE_ENABLED = false; // distribute captures by equal path distance instead of waypoint-to-waypoint
constexpr uint16_t FLOWLAPSE_FRAME_COUNT_TARGET = 48; // total captures including first and last stop in frame-count mode
constexpr bool FLOWLAPSE_FRAMECOUNT_AUTO_EXIT = true; // exit frame-count mode after completion when enabled
constexpr uint16_t FLOWLAPSE_NORMALIZED_LUT_SCALE = 65535U;
constexpr uint8_t FLOWLAPSE_LENGTH_STORAGE_SCALE = 8U; // stored path values use 1/8-unit resolution

struct PersistedSettingsV1 {
  uint16_t magic;
  uint8_t version;
  uint8_t timelapseIntervalSeconds;
  uint8_t checksum;
};

struct PersistedSettingsV2 {
  uint16_t magic;
  uint8_t version;
  uint8_t timelapseIntervalSeconds;
  int stepDist;
  uint8_t checksum;
};

struct PersistedSettingsV3 {
  uint16_t magic;
  uint8_t version;
  uint8_t timelapseIntervalSeconds;
  int stepDist;
  uint8_t flags;
  uint8_t checksum;
};

struct PersistedSettingsV4 {
  uint16_t magic;
  uint8_t version;
  uint8_t timelapseIntervalSeconds;
  int stepDist;
  uint8_t flags;
  float homeSwing;
  float homeLift;
  float homePan;
  float homeTilt;
  uint8_t checksum;
};

struct PersistedSettingsV5 {
  uint16_t magic;
  uint8_t version;
  uint8_t timelapseIntervalSeconds;
  int stepDist;
  uint16_t timelapseSettleDwellMs;
  uint8_t flags;
  float homeSwing;
  float homeLift;
  float homePan;
  float homeTilt;
  uint8_t checksum;
};

struct FlowlapseWaypoint {
  float swing;
  float lift;
  float pan;
  float tilt;
  uint16_t dwellMs;
};

enum FlowlapseState : uint8_t {
  FLOWLAPSE_STATE_RECORDING,
  FLOWLAPSE_STATE_READY_FOR_PREVIEW,
  FLOWLAPSE_STATE_PREVIEW_RUNNING,
  FLOWLAPSE_STATE_READY_FOR_CAPTURE,
  FLOWLAPSE_STATE_CAPTURE_RUNNING,
  FLOWLAPSE_STATE_CAPTURE_PAUSED,
  FLOWLAPSE_STATE_UNDO_RUNNING,
  FLOWLAPSE_STATE_UNDO_READY_DELETE,
  FLOWLAPSE_STATE_JOG_RUNNING
};

enum FlowlapseCapturePhase : uint8_t {
  FLOWLAPSE_CAPTURE_TRIGGER_LOW,
  FLOWLAPSE_CAPTURE_TRIGGER_HIGH,
  FLOWLAPSE_CAPTURE_MOVE_ACTIVE,
  FLOWLAPSE_CAPTURE_DWELL
};

enum FlowlapseAccelerationProfile : uint8_t {
  FLOWLAPSE_ACCEL_LINEAR = 0,
  FLOWLAPSE_ACCEL_EASE_IN_OUT,
  FLOWLAPSE_ACCEL_CINEMATIC_SLOW_START,
  FLOWLAPSE_ACCEL_ROBOTIC_CONSTANT,
  FLOWLAPSE_ACCEL_PROFILE_COUNT
};

constexpr uint8_t MOTION_FLAG_SWING = 0x01;
constexpr uint8_t MOTION_FLAG_LIFT = 0x02;
constexpr uint8_t DRONE_AXIS_LAST_SWING = 0x01;
constexpr uint8_t DRONE_AXIS_LAST_LIFT = 0x02;
constexpr uint8_t DRONE_AXIS_LAST_PAN = 0x04;
constexpr uint8_t DRONE_AXIS_LAST_TILT = 0x08;

inline bool isPackedStateSet(uint8_t flags, uint8_t mask) {
  return (flags & mask) != 0;
}

inline void setPackedState(uint8_t& flags, uint8_t mask, bool enabled) {
  if (enabled) {
    flags |= mask;
  } else {
    flags &= static_cast<uint8_t>(~mask);
  }
}

struct PackedFlags {
  uint8_t isSwingReversed : 1;
  uint8_t isPanReversed : 1;
  uint8_t isLiftReversed : 1;
  uint8_t isTiltReversed : 1;
  uint8_t isFocusReversed : 1;
  uint8_t droneMode : 1;
  uint8_t lastIntervalAdjustUpComboActive : 1;
  uint8_t lastIntervalAdjustDownComboActive : 1;

  uint8_t lastStepDistAdjustUpComboActive : 1;
  uint8_t lastStepDistAdjustDownComboActive : 1;
  uint8_t lastEmergencyStopComboActive : 1;
  uint8_t lastRumbleMuteToggleComboActive : 1;
  uint8_t rumbleMuted : 1;
  uint8_t suppressNextSelectRelease : 1;
  uint8_t suppressNextStartRelease : 1;
  uint8_t lastSettingsReplayComboActive : 1;

  uint8_t lastDronePrecisionModeActive : 1;
  uint8_t lastDroneBoostModeActive : 1;
  uint8_t lastFlowlapseClearComboActive : 1;
  uint8_t lastFlowlapseDeleteLastComboActive : 1;
  uint8_t lastFlowlapseFrameModeToggleComboActive : 1;
  uint8_t lastFlowlapseReturnToStartComboActive : 1;
  uint8_t lastDwellAdjustUpComboActive : 1;
  uint8_t lastDwellAdjustDownComboActive : 1;
  uint8_t suppressDroneNextSelectRelease : 1;

  uint8_t suppressDroneNextStartRelease : 1;
  uint8_t suppressDroneNextL3Release : 1;
  uint8_t flowlapseL3HoldActive : 1;
  uint8_t flowlapseCaptureAlignedToFirstWaypoint : 1;
  uint8_t flowlapseArcLengthLutValid : 1;
  uint8_t flowlapseFrameCountModeEnabled : 1;
  uint8_t flowlapseFrameCountModeActive : 1;
  uint8_t flowlapsePreviewFrameModeActive : 1;

  uint8_t unsupportedControllerWarningShown : 1;
  uint8_t rumbleSeparatorActive : 1;
  uint8_t chainStepDistAfterInterval : 1;
  uint8_t settingsMode : 1;

  PackedFlags()
    : isSwingReversed(false),
      isPanReversed(false),
      isLiftReversed(false),
      isTiltReversed(false),
      isFocusReversed(false),
      droneMode(false),
      lastIntervalAdjustUpComboActive(false),
      lastIntervalAdjustDownComboActive(false),
      lastStepDistAdjustUpComboActive(false),
      lastStepDistAdjustDownComboActive(false),
      lastEmergencyStopComboActive(false),
      lastRumbleMuteToggleComboActive(false),
      rumbleMuted(false),
      suppressNextSelectRelease(false),
      suppressNextStartRelease(false),
      lastSettingsReplayComboActive(false),
      lastDronePrecisionModeActive(false),
      lastDroneBoostModeActive(false),
      lastFlowlapseClearComboActive(false),
      lastFlowlapseDeleteLastComboActive(false),
      lastFlowlapseFrameModeToggleComboActive(false),
      lastFlowlapseReturnToStartComboActive(false),
      lastDwellAdjustUpComboActive(false),
      lastDwellAdjustDownComboActive(false),
      suppressDroneNextSelectRelease(false),
      suppressDroneNextStartRelease(false),
      suppressDroneNextL3Release(false),
      flowlapseL3HoldActive(false),
      flowlapseCaptureAlignedToFirstWaypoint(false),
      flowlapseArcLengthLutValid(false),
      flowlapseFrameCountModeEnabled(FLOWLAPSE_FRAME_COUNT_MODE_ENABLED),
      flowlapseFrameCountModeActive(false),
      flowlapsePreviewFrameModeActive(false),
      unsupportedControllerWarningShown(false),
      rumbleSeparatorActive(false),
      chainStepDistAfterInterval(false),
      settingsMode(false) {}
};

PackedFlags packedFlags;

#define isSwingReversed packedFlags.isSwingReversed
#define isPanReversed packedFlags.isPanReversed
#define isLiftReversed packedFlags.isLiftReversed
#define isTiltReversed packedFlags.isTiltReversed
#define isFocusReversed packedFlags.isFocusReversed
#define droneMode packedFlags.droneMode
#define lastIntervalAdjustUpComboActive packedFlags.lastIntervalAdjustUpComboActive
#define lastIntervalAdjustDownComboActive packedFlags.lastIntervalAdjustDownComboActive
#define lastStepDistAdjustUpComboActive packedFlags.lastStepDistAdjustUpComboActive
#define lastStepDistAdjustDownComboActive packedFlags.lastStepDistAdjustDownComboActive
#define lastEmergencyStopComboActive packedFlags.lastEmergencyStopComboActive
#define lastRumbleMuteToggleComboActive packedFlags.lastRumbleMuteToggleComboActive
#define rumbleMuted packedFlags.rumbleMuted
#define suppressNextSelectRelease packedFlags.suppressNextSelectRelease
#define suppressNextStartRelease packedFlags.suppressNextStartRelease
#define lastSettingsReplayComboActive packedFlags.lastSettingsReplayComboActive
#define lastDronePrecisionModeActive packedFlags.lastDronePrecisionModeActive
#define lastDroneBoostModeActive packedFlags.lastDroneBoostModeActive
bool lastDroneSwingActive = false;
bool lastDroneLiftActive = false;
bool lastDronePanActive = false;
bool lastDroneTiltActive = false;
int8_t lastDroneSwingDirection = 0;
int8_t lastDroneLiftDirection = 0;
int8_t lastDronePanDirection = 0;
int8_t lastDroneTiltDirection = 0;
bool lastWaypointDwellAdjustUpComboActive = false;
bool lastWaypointDwellAdjustDownComboActive = false;
bool lastFlowlapseLoopToggleComboActive = false;
bool lastSelectButtonState = false;
bool lastStartButtonState = false;
bool lastTriangleButtonState = false;
bool lastL3ButtonState = false;
bool lastL3SettingsState = false;
bool lastPadRightButtonState = false;
bool lastPadLeftButtonState = false;
bool lastDroneSelectButtonState = false;
bool lastDroneStartButtonState = false;
bool lastR3ButtonState = false;
bool lastHomeSetComboActive = false;
bool lastHomeGoComboActive = false;
bool lastHomeClearComboActive = false;
bool lastTimelapseSettleAdjustUpComboActive = false;
bool lastTimelapseSettleAdjustDownComboActive = false;
bool lastTimelapseMaxSpeedStageAdjustUpComboActive = false;
bool lastTimelapseMaxSpeedStageAdjustDownComboActive = false;
bool lastTimelapseAntiBacklashToggleComboActive = false;
#define lastFlowlapseClearComboActive packedFlags.lastFlowlapseClearComboActive
#define lastFlowlapseDeleteLastComboActive packedFlags.lastFlowlapseDeleteLastComboActive
#define lastFlowlapseFrameModeToggleComboActive packedFlags.lastFlowlapseFrameModeToggleComboActive
#define lastFlowlapseReturnToStartComboActive packedFlags.lastFlowlapseReturnToStartComboActive
#define lastDwellAdjustUpComboActive packedFlags.lastDwellAdjustUpComboActive
#define lastDwellAdjustDownComboActive packedFlags.lastDwellAdjustDownComboActive
#define suppressDroneNextSelectRelease packedFlags.suppressDroneNextSelectRelease
#define suppressDroneNextStartRelease packedFlags.suppressDroneNextStartRelease
#define suppressDroneNextL3Release packedFlags.suppressDroneNextL3Release
#define flowlapseL3HoldActive packedFlags.flowlapseL3HoldActive
#define swingInMotion (isPackedStateSet(motionFlags, MOTION_FLAG_SWING) || swingSoloMode != 0)
#define liftInMotion (isPackedStateSet(motionFlags, MOTION_FLAG_LIFT) || liftSoloMode != 0)
unsigned long flowlapseL3HoldStartMs = 0;
unsigned long flowlapseDwellMs = FLOWLAPSE_WAYPOINT_DWELL_MS;
bool flowlapsePingPongLoopEnabled = FLOWLAPSE_PING_PONG_LOOP;
bool droneProportionalStickSpeedEnabled = false;
FlowlapseAccelerationProfile flowlapseAccelerationProfile =
  FLOWLAPSE_EASE_IN_OUT_ENABLED ? FLOWLAPSE_ACCEL_EASE_IN_OUT : FLOWLAPSE_ACCEL_LINEAR;
unsigned long droneLastActivityMs = 0;
FlowlapseWaypoint flowlapseWaypoints[FLOWLAPSE_MAX_WAYPOINTS];
uint8_t flowlapseWaypointCount = 0;
FlowlapseState flowlapseState = FLOWLAPSE_STATE_RECORDING;
FlowlapseCapturePhase flowlapseCapturePhase = FLOWLAPSE_CAPTURE_TRIGGER_LOW;
unsigned long flowlapseCapturePhaseStartMs = 0;
unsigned long flowlapseCaptureDwellMs = 0;
unsigned long flowlapsePreviewHoldUntilMs = 0;
unsigned long flowlapseLastUpdateMs = 0;
unsigned long flowlapseLastProgressLogMs = 0;
unsigned long flowlapseCaptureRunStartMs = 0;
unsigned long flowlapsePauseStartMs = 0;
unsigned long flowlapseTotalPausedMs = 0;
uint16_t flowlapsePingPongEdgeCount = 0;
uint8_t flowlapseTargetWaypointIndex = 0;
uint8_t flowlapseJogIndex = 0;
#define flowlapseCaptureAlignedToFirstWaypoint packedFlags.flowlapseCaptureAlignedToFirstWaypoint
int8_t flowlapseCaptureDirection = 1;
#define flowlapseArcLengthLutValid packedFlags.flowlapseArcLengthLutValid
uint8_t flowlapseArcLengthLutSegmentBaseIndex = 255;
uint16_t flowlapseArcLengthLutNormalized[FLOWLAPSE_ARC_LENGTH_TABLE_STEPS + 1];
#define flowlapseFrameCountModeEnabled packedFlags.flowlapseFrameCountModeEnabled
#define flowlapseFrameCountModeActive packedFlags.flowlapseFrameCountModeActive
uint16_t flowlapseFrameCountTarget = 0;
uint16_t flowlapseFrameCountCurrentStopIndex = 0;
uint16_t flowlapseFrameCountSpacingStored = 0;
#define flowlapsePreviewFrameModeActive packedFlags.flowlapsePreviewFrameModeActive
uint16_t flowlapsePreviewFrameTarget = 0;
uint16_t flowlapsePreviewFrameStopIndex = 0;
uint16_t flowlapsePathTotalLengthStored = 0;
uint16_t flowlapsePathSegmentCumulative[FLOWLAPSE_MAX_WAYPOINTS - 1];

bool homePoseValid = false;
bool homeReturnActive = false;
FlowlapseWaypoint homePose = {0.0f, 0.0f, 0.0f, 0.0f, 0};

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
uint8_t droneManualSwingTier = DEFAULT_AXIS_SPEED_STAGE;
uint8_t droneManualLiftTier = DEFAULT_AXIS_SPEED_STAGE;
uint8_t droneManualPanTier = DEFAULT_AXIS_SPEED_STAGE;
uint8_t droneManualTiltTier = DEFAULT_AXIS_SPEED_STAGE;
unsigned long droneManualSwingTierLastStepMs = 0;
unsigned long droneManualLiftTierLastStepMs = 0;
unsigned long droneManualPanTierLastStepMs = 0;
unsigned long droneManualTiltTierLastStepMs = 0;
unsigned long lastDroneUiBroadcastMs = 0;
int8_t lastBroadcastSwing = 0;
int8_t lastBroadcastLift = 0;
int8_t lastBroadcastPan = 0;
int8_t lastBroadcastTilt = 0;
bool lastBroadcastLeftStickClick = false;
bool lastBroadcastRightStickClick = false;
unsigned long r3PressStartMs = 0;

unsigned long lastControllerRetryMs = 0;
unsigned long lastControllerOkBroadcastMs = 0;
unsigned long lastLoopBypassLogMs = 0;
char lastLoopBypassReason[32] = "";
#define unsupportedControllerWarningShown packedFlags.unsupportedControllerWarningShown
#define rumbleSeparatorActive packedFlags.rumbleSeparatorActive
#define settingsMode packedFlags.settingsMode
#define chainStepDistAfterInterval packedFlags.chainStepDistAfterInterval
bool persistedSettingsResetComboActive = false;
bool persistedSettingsResetLatched = false;
unsigned long persistedSettingsResetStartMs = 0;

void configureController() {

  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, PRESSURES, RUMBLE);

  if (error == 0) {
    Serial.print(F("Controller configured. pressures="));
    Serial.print(PRESSURES ? "true" : "false");
    Serial.print(F(", rumble="));
    Serial.println(RUMBLE ? "true" : "false");
  }
  else if (error == 1)
    Serial.println(F("No controller found, check wiring."));
  else if (error == 2)
    Serial.println(F("Controller found but not accepting commands."));
  else if (error == 3)
    Serial.println(F("Controller refusing Pressures mode, may not support it."));
}

void detectControllerType() {

  controllerType = ps2x.readType();
  switch (controllerType) {
    case 0:
      Serial.println(F("Unknown Controller type found"));
      break;
    case 1:
      Serial.println(F("DualShock Controller found"));
      break;
    case 2:
      Serial.println(F("GuitarHero Controller found"));
      break;
    case 3:
      Serial.println(F("Wireless Sony DualShock Controller found"));
      break;
  }
}

void sendToDisplayESP(const char* msg) {
  if (Serial1) {
    Serial1.println(msg);
  }
}

void sendToRGBESP(const char* msg) {
  if (Serial2) {
    Serial2.println(msg);
  }
}

void broadcastStatus(const char* msg) {
  sendToDisplayESP(msg);
  sendToRGBESP(msg);
}

void broadcastFlowlapseWaypointCount() {
  char waypointCountLine[48];
  snprintf(waypointCountLine, sizeof(waypointCountLine), "WAYPOINT_COUNT:%u/%u",
           static_cast<unsigned int>(flowlapseWaypointCount),
           static_cast<unsigned int>(FLOWLAPSE_MAX_WAYPOINTS));
  broadcastStatus(waypointCountLine);
}

void emitSpeedTestEvent(const char* eventTag, const char* actionTag = "PRESS") {
  if (!WILL_TEST_SPEED_BUILD) {
    return;
  }

  char speedEventLine[64];
  snprintf(speedEventLine, sizeof(speedEventLine), "SPEED_EVENT:%s:%s", eventTag, actionTag);
  Serial.println(speedEventLine);
  broadcastStatus(speedEventLine);

  digitalWrite(LED_BUILTIN, HIGH);
  delay(WILL_TEST_LED_PULSE_MS);
  digitalWrite(LED_BUILTIN, LOW);
}

void emitControlIndicator(const char* controlTag) {
  char controlLine[64];
  snprintf(controlLine, sizeof(controlLine), "CONTROL:%s", controlTag);
  Serial.println(controlLine);
  broadcastStatus(controlLine);
}

void emitSpeedInputEdge(const char* buttonTag, const char* actionTag) {
  if (!WILL_TEST_SPEED_BUILD) {
    return;
  }

  char speedInputLine[72];
  snprintf(speedInputLine, sizeof(speedInputLine), "SPEED_INPUT:%s:%s:%s",
           buttonTag,
           actionTag,
           droneMode ? "DRONE" : "REGULAR");
  Serial.println(speedInputLine);
  broadcastStatus(speedInputLine);
}

void logShoulderSpeedButtonEdges() {
  if (ps2x.ButtonPressed(PSB_L1)) {
    emitSpeedInputEdge("L1", "PRESS");
  }
  if (ps2x.ButtonReleased(PSB_L1)) {
    emitSpeedInputEdge("L1", "RELEASE");
  }

  if (ps2x.ButtonPressed(PSB_L2)) {
    emitSpeedInputEdge("L2", "PRESS");
  }
  if (ps2x.ButtonReleased(PSB_L2)) {
    emitSpeedInputEdge("L2", "RELEASE");
  }

  if (ps2x.ButtonPressed(PSB_R1)) {
    emitSpeedInputEdge("R1", "PRESS");
  }
  if (ps2x.ButtonReleased(PSB_R1)) {
    emitSpeedInputEdge("R1", "RELEASE");
  }

  if (ps2x.ButtonPressed(PSB_R2)) {
    emitSpeedInputEdge("R2", "PRESS");
  }
  if (ps2x.ButtonReleased(PSB_R2)) {
    emitSpeedInputEdge("R2", "RELEASE");
  }
}

void emitLoopBypassReason(const char* reasonTag) {
  if (!WILL_TEST_SPEED_BUILD || !WILL_TEST_BYPASS_LOGS) {
    return;
  }

  const unsigned long now = millis();
  const bool sameReason = (strncmp(lastLoopBypassReason, reasonTag, sizeof(lastLoopBypassReason)) == 0);
  if (sameReason && (now - lastLoopBypassLogMs) < 750) {
    return;
  }

  strncpy(lastLoopBypassReason, reasonTag, sizeof(lastLoopBypassReason) - 1);
  lastLoopBypassReason[sizeof(lastLoopBypassReason) - 1] = '\0';
  lastLoopBypassLogMs = now;

  char bypassLine[72];
  snprintf(bypassLine, sizeof(bypassLine), "SPEED_PATH_BYPASS:%s", reasonTag);
  Serial.println(bypassLine);
  broadcastStatus(bypassLine);
}

void handleAxisSpeedControl(uint16_t buttonCode, uint8_t axis1Pin, uint8_t axis2Pin, const char* eventTag) {
  const bool held    = ps2x.Button(buttonCode);
  const bool pressed = ps2x.ButtonPressed(buttonCode);
  const bool released = ps2x.ButtonReleased(buttonCode);

  if (WILL_TEST_LEGACY_HELD_SPEED_SIGNALS) {
    if (held) {
      digitalWrite(axis1Pin, HIGH);
      digitalWrite(axis2Pin, HIGH);
    } else {
      digitalWrite(axis1Pin, LOW);
      digitalWrite(axis2Pin, LOW);
    }
    if (pressed) {
      emitSpeedTestEvent(eventTag, "PRESS");
    }
    if (released) {
      emitSpeedTestEvent(eventTag, "RELEASE");
    }
    return;
  }

  if (pressed) {
    pulseSpeedStageUpPin(axis1Pin);
    pulseSpeedStageUpPin(axis2Pin);
    emitControlIndicator(eventTag);
    emitSpeedTestEvent(eventTag, "PRESS");
  } else if (released) {
    digitalWrite(axis1Pin, LOW);
    digitalWrite(axis2Pin, LOW);
    emitSpeedTestEvent(eventTag, "RELEASE");
  }
}

void pulseSpeedStageUpPin(uint8_t speedUpPin) {
  digitalWrite(speedUpPin, HIGH);
  delay(SPEED_STAGE_PULSE_HIGH_MS);
  digitalWrite(speedUpPin, LOW);
  delay(SPEED_STAGE_PULSE_LOW_MS);
}

void pulseDroneSpeedStagePin(uint8_t speedPin) {
  digitalWrite(speedPin, HIGH);
  delay(DRONE_SPEED_STAGE_PULSE_HIGH_MS);
  digitalWrite(speedPin, LOW);
  delay(DRONE_SPEED_STAGE_PULSE_LOW_MS);
}

void pulseAxisTierToward(uint8_t& currentTier, uint8_t targetTier, uint8_t speedUpPin, uint8_t speedDownPin) {
  if (currentTier < targetTier) {
    pulseSpeedStageUpPin(speedUpPin);
    currentTier++;
  } else if (currentTier > targetTier) {
    pulseSpeedStageUpPin(speedDownPin);
    currentTier--;
  }
}

void applyDefaultAxisSpeedStage(uint8_t speedUpPin, uint8_t targetStage) {
  uint8_t clampedStage = static_cast<uint8_t>(constrain(static_cast<int>(targetStage), 0, 4));
  for (uint8_t i = 0; i < clampedStage; ++i) {
    pulseSpeedStageUpPin(speedUpPin);
  }
}

void applyStartupDefaultSpeedStages() {
  applyDefaultAxisSpeedStage(swingSpeedUp, DEFAULT_AXIS_SPEED_STAGE);
  applyDefaultAxisSpeedStage(liftSpeedUp, DEFAULT_AXIS_SPEED_STAGE);
  applyDefaultAxisSpeedStage(panSpeedUp, DEFAULT_AXIS_SPEED_STAGE);
  applyDefaultAxisSpeedStage(tiltSpeedUp, DEFAULT_AXIS_SPEED_STAGE);
}

void pulseAllMotionAxisSpeedPins(uint8_t swingPin, uint8_t panPin, uint8_t liftPin, uint8_t tiltPin) {
  digitalWrite(swingPin, HIGH);
  digitalWrite(panPin, HIGH);
  digitalWrite(liftPin, HIGH);
  digitalWrite(tiltPin, HIGH);
  delay(SPEED_STAGE_PULSE_HIGH_MS);

  digitalWrite(swingPin, LOW);
  digitalWrite(panPin, LOW);
  digitalWrite(liftPin, LOW);
  digitalWrite(tiltPin, LOW);
  delay(SPEED_STAGE_PULSE_LOW_MS);
}

void normalizeMotionAxisSpeedStages(uint8_t targetStage) {
  constexpr uint8_t NANO_AXIS_MAX_STAGE = 4;
  uint8_t clampedTargetStage = static_cast<uint8_t>(constrain(static_cast<int>(targetStage), 0, NANO_AXIS_MAX_STAGE));

  // Drive all motion axes to stage 0 first, then move to the requested stage.
  for (uint8_t i = 0; i < NANO_AXIS_MAX_STAGE; ++i) {
    pulseAllMotionAxisSpeedPins(swingSpeedDown, panSpeedDown, liftSpeedDown, tiltSpeedDown);
  }

  for (uint8_t i = 0; i < clampedTargetStage; ++i) {
    pulseAllMotionAxisSpeedPins(swingSpeedUp, panSpeedUp, liftSpeedUp, tiltSpeedUp);
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
  bool boostModeActive = DRONE_ENABLE_BOOST_MODIFIER
      && ps2x.Button(PSB_R2)
      && !(DRONE_L2_PRIORITY_OVER_BOOST && precisionModeActive);

  // Precision has priority over boost when configured.
  if (precisionModeActive && !boostModeActive) {
    return DRONE_SPEED_TIER_STOP;  // Return STOP; actual motion is gated by proportional magnitude (stick position)
  }

  // Boost mode (when not suppressed by precision priority)
  if (boostModeActive && speedTier < maxAllowedTier) {
    speedTier = static_cast<uint8_t>(min(
        static_cast<int>(maxAllowedTier),
        static_cast<int>(speedTier) + static_cast<int>(DRONE_BOOST_TIER_DELTA)));
  }

  return speedTier;
}

uint8_t getProportionalTargetSpeedTier(int magnitude, uint8_t maxSpeedTier, uint8_t expoPercent) {
  bool microMotionActive = DRONE_ENABLE_PRECISION_MODIFIER && ps2x.Button(PSB_L2);

  uint8_t speedTier = getProportionalSpeedTier(magnitude, expoPercent);
  uint8_t clampedMaxTier = static_cast<uint8_t>(constrain(static_cast<int>(maxSpeedTier), DRONE_SPEED_TIER_STOP, DRONE_SPEED_TIER_MAX));

  if (speedTier > clampedMaxTier) {
    speedTier = clampedMaxTier;
  }

  // Precision micro-motion: duty-cycle MED tier using stick deflection and configured ratio.
  if (microMotionActive && magnitude > 0) {
    constexpr unsigned long DRONE_MICRO_MOTION_CYCLE_MS = 120UL;

    int expoMagnitude = getExpoDeflectionMagnitude(magnitude, expoPercent);
    float normalizedDeflection = static_cast<float>(expoMagnitude) / static_cast<float>(DRONE_STICK_MAX_DEFLECTION);
    if (normalizedDeflection < 0.0f) normalizedDeflection = 0.0f;
    if (normalizedDeflection > 1.0f) normalizedDeflection = 1.0f;

    float dutyRatio = DRONE_MICRO_MOTION_SPEED_RATIO * normalizedDeflection;
    if (dutyRatio < 0.01f) {
      return DRONE_SPEED_TIER_STOP;
    }
    if (dutyRatio > 0.95f) dutyRatio = 0.95f;

    unsigned long onWindowMs = static_cast<unsigned long>(DRONE_MICRO_MOTION_CYCLE_MS * dutyRatio);
    if (onWindowMs == 0UL) {
      onWindowMs = 1UL;
    }

    bool microPulseOn = (millis() % DRONE_MICRO_MOTION_CYCLE_MS) < onWindowMs;
    return microPulseOn ? DRONE_SPEED_TIER_MED : DRONE_SPEED_TIER_STOP;
  }

  return applyDroneSpeedTierModifiers(speedTier, clampedMaxTier);
}

void stepDroneAxisTierTowardTarget(uint8_t& currentTier,
                                   uint8_t targetTier,
                                   uint8_t speedUpPin,
                                   uint8_t speedDownPin,
                                   unsigned long& lastStepMs,
                                   unsigned long now) {
  uint8_t clampedTargetTier = static_cast<uint8_t>(constrain(static_cast<int>(targetTier), DRONE_SPEED_TIER_STOP, DRONE_SPEED_TIER_MAX));

  if (currentTier == clampedTargetTier) {
    return;
  }

  if (now - lastStepMs < DRONE_MANUAL_TIER_STEP_INTERVAL_MS) {
    return;
  }

  uint8_t oldTier = currentTier;
  if (currentTier < clampedTargetTier) {
    pulseDroneSpeedStagePin(speedUpPin);
    currentTier++;
  } else {
    pulseDroneSpeedStagePin(speedDownPin);
    currentTier--;
  }

  if (DEBUG_DRONE_TIERS) {
    Serial.print(F("DRONE TIER: "));
    Serial.print(oldTier);
    Serial.print(F(" -> "));
    Serial.print(currentTier);
    Serial.print(F(" (target "));
    Serial.print(clampedTargetTier);
    Serial.println(F(")"));
  }

  lastStepMs = now;
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

float clampFlowlapse01(float value) {
  if (value < 0.0f) {
    return 0.0f;
  }
  if (value > 1.0f) {
    return 1.0f;
  }
  return value;
}

uint16_t encodeFlowlapseNormalizedValue(float value) {
  float clampedValue = clampFlowlapse01(value);
  return static_cast<uint16_t>(clampedValue * static_cast<float>(FLOWLAPSE_NORMALIZED_LUT_SCALE) + 0.5f);
}

float decodeFlowlapseNormalizedValue(uint16_t value) {
  return static_cast<float>(value) / static_cast<float>(FLOWLAPSE_NORMALIZED_LUT_SCALE);
}

uint16_t encodeFlowlapseStoredLength(float value) {
  if (value <= 0.0f) {
    return 0;
  }

  float scaledValue = value * static_cast<float>(FLOWLAPSE_LENGTH_STORAGE_SCALE);
  if (scaledValue >= 65535.0f) {
    return 65535U;
  }

  return static_cast<uint16_t>(scaledValue + 0.5f);
}

float decodeFlowlapseStoredLength(uint16_t value) {
  return static_cast<float>(value) / static_cast<float>(FLOWLAPSE_LENGTH_STORAGE_SCALE);
}

float getFlowlapsePathTotalLength() {
  return decodeFlowlapseStoredLength(flowlapsePathTotalLengthStored);
}

float getFlowlapseFrameCountSpacing() {
  return decodeFlowlapseStoredLength(flowlapseFrameCountSpacingStored);
}

float getFlowlapsePathSegmentLength(uint8_t segmentIndex) {
  if (segmentIndex >= FLOWLAPSE_MAX_WAYPOINTS - 1) {
    return 0.0f;
  }

  uint16_t segmentEndStored = flowlapsePathSegmentCumulative[segmentIndex];
  uint16_t segmentStartStored = (segmentIndex == 0) ? 0 : flowlapsePathSegmentCumulative[segmentIndex - 1];
  if (segmentEndStored <= segmentStartStored) {
    return 0.0f;
  }

  return decodeFlowlapseStoredLength(static_cast<uint16_t>(segmentEndStored - segmentStartStored));
}

const char* getFlowlapseAccelerationProfileName(FlowlapseAccelerationProfile profile) {
  switch (profile) {
    case FLOWLAPSE_ACCEL_LINEAR: return "linear";
    case FLOWLAPSE_ACCEL_EASE_IN_OUT: return "ease in/out";
    case FLOWLAPSE_ACCEL_CINEMATIC_SLOW_START: return "cinematic slow start";
    case FLOWLAPSE_ACCEL_ROBOTIC_CONSTANT: return "robotic constant";
    default: return "unknown";
  }
}

float applyFlowlapseEaseInOut(float t) {
  float clampedT = clampFlowlapse01(t);

  switch (flowlapseAccelerationProfile) {
    case FLOWLAPSE_ACCEL_LINEAR:
      return clampedT;

    case FLOWLAPSE_ACCEL_EASE_IN_OUT: {
      float smoothStep = clampedT * clampedT * (3.0f - 2.0f * clampedT);
      float easedStrength = clampFlowlapse01(FLOWLAPSE_EASE_STRENGTH);
      return clampedT + (smoothStep - clampedT) * easedStrength;
    }

    case FLOWLAPSE_ACCEL_CINEMATIC_SLOW_START: {
      float easedStrength = clampFlowlapse01(FLOWLAPSE_EASE_STRENGTH);
      float easeInCubic = clampedT * clampedT * clampedT;
      return clampedT + (easeInCubic - clampedT) * easedStrength;
    }

    case FLOWLAPSE_ACCEL_ROBOTIC_CONSTANT: {
      constexpr float kStepCount = 6.0f;
      if (clampedT >= 1.0f) {
        return 1.0f;
      }
      return floor(clampedT * kStepCount) / kStepCount;
    }

    default:
      return clampedT;
  }
}

FlowlapseWaypoint interpolateFlowlapseLinearPoint(uint8_t segmentStartIndex, float t) {
  uint8_t startIndex = segmentStartIndex;
  uint8_t endIndex = static_cast<uint8_t>(segmentStartIndex + 1);

  const FlowlapseWaypoint& start = flowlapseWaypoints[startIndex];
  const FlowlapseWaypoint& end = flowlapseWaypoints[endIndex];

  float clampedT = clampFlowlapse01(t);
  FlowlapseWaypoint result;
  result.swing = start.swing + (end.swing - start.swing) * clampedT;
  result.lift  = start.lift  + (end.lift  - start.lift)  * clampedT;
  result.pan   = start.pan   + (end.pan   - start.pan)   * clampedT;
  result.tilt  = start.tilt  + (end.tilt  - start.tilt)  * clampedT;
  return result;
}

FlowlapseWaypoint interpolateFlowlapseLinearPointBetween(uint8_t startIndex, uint8_t endIndex, float t) {
  const FlowlapseWaypoint& start = flowlapseWaypoints[startIndex];
  const FlowlapseWaypoint& end = flowlapseWaypoints[endIndex];

  float clampedT = clampFlowlapse01(t);
  FlowlapseWaypoint result;
  result.swing = start.swing + (end.swing - start.swing) * clampedT;
  result.lift  = start.lift  + (end.lift  - start.lift)  * clampedT;
  result.pan   = start.pan   + (end.pan   - start.pan)   * clampedT;
  result.tilt  = start.tilt  + (end.tilt  - start.tilt)  * clampedT;
  return result;
}

float interpolateCatmullRomScalar(float p0, float p1, float p2, float p3, float t) {
  float t2 = t * t;
  float t3 = t2 * t;
  return 0.5f * ((2.0f * p1)
      + (-p0 + p2) * t
      + (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2
      + (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3);
}

FlowlapseWaypoint interpolateFlowlapseCurvedPoint(uint8_t segmentStartIndex, float t) {
  uint8_t p1Index = segmentStartIndex;
  uint8_t p2Index = static_cast<uint8_t>(segmentStartIndex + 1);
  uint8_t p0Index = (p1Index > 0) ? static_cast<uint8_t>(p1Index - 1) : p1Index;
  uint8_t p3Index = (p2Index + 1 < flowlapseWaypointCount) ? static_cast<uint8_t>(p2Index + 1) : p2Index;

  const FlowlapseWaypoint& p0 = flowlapseWaypoints[p0Index];
  const FlowlapseWaypoint& p1 = flowlapseWaypoints[p1Index];
  const FlowlapseWaypoint& p2 = flowlapseWaypoints[p2Index];
  const FlowlapseWaypoint& p3 = flowlapseWaypoints[p3Index];

  float clampedT = clampFlowlapse01(t);
  FlowlapseWaypoint result;
  result.swing = interpolateCatmullRomScalar(p0.swing, p1.swing, p2.swing, p3.swing, clampedT);
  result.lift  = interpolateCatmullRomScalar(p0.lift,  p1.lift,  p2.lift,  p3.lift,  clampedT);
  result.pan   = interpolateCatmullRomScalar(p0.pan,   p1.pan,   p2.pan,   p3.pan,   clampedT);
  result.tilt  = interpolateCatmullRomScalar(p0.tilt,  p1.tilt,  p2.tilt,  p3.tilt,  clampedT);
  return result;
}

float computeFlowlapseWaypointDistance(const FlowlapseWaypoint& a, const FlowlapseWaypoint& b) {
  float deltaSwing = b.swing - a.swing;
  float deltaLift = b.lift - a.lift;
  float deltaPan = b.pan - a.pan;
  float deltaTilt = b.tilt - a.tilt;
  return sqrt(deltaSwing * deltaSwing + deltaLift * deltaLift + deltaPan * deltaPan + deltaTilt * deltaTilt);
}

void invalidateFlowlapseArcLengthLut() {
  flowlapseArcLengthLutValid = false;
  flowlapseArcLengthLutSegmentBaseIndex = 255;
}

void buildFlowlapseArcLengthLut(uint8_t segmentBaseIndex) {
  flowlapseArcLengthLutSegmentBaseIndex = segmentBaseIndex;
  flowlapseArcLengthLutNormalized[0] = 0;

  FlowlapseWaypoint previousSample = interpolateFlowlapseCurvedPoint(segmentBaseIndex, 0.0f);
  float accumulatedLength = 0.0f;

  for (uint8_t i = 1; i <= FLOWLAPSE_ARC_LENGTH_TABLE_STEPS; ++i) {
    float sampleT = static_cast<float>(i) / static_cast<float>(FLOWLAPSE_ARC_LENGTH_TABLE_STEPS);
    FlowlapseWaypoint sample = interpolateFlowlapseCurvedPoint(segmentBaseIndex, sampleT);
    accumulatedLength += computeFlowlapseWaypointDistance(previousSample, sample);
    flowlapseArcLengthLutNormalized[i] = encodeFlowlapseStoredLength(accumulatedLength);
    previousSample = sample;
  }

  if (accumulatedLength <= 0.0001f) {
    for (uint8_t i = 0; i <= FLOWLAPSE_ARC_LENGTH_TABLE_STEPS; ++i) {
      flowlapseArcLengthLutNormalized[i] = encodeFlowlapseNormalizedValue(
          static_cast<float>(i) / static_cast<float>(FLOWLAPSE_ARC_LENGTH_TABLE_STEPS));
    }
  } else {
    flowlapseArcLengthLutNormalized[0] = 0;
    for (uint8_t i = 1; i <= FLOWLAPSE_ARC_LENGTH_TABLE_STEPS; ++i) {
      float normalizedDistance = decodeFlowlapseStoredLength(flowlapseArcLengthLutNormalized[i]) / accumulatedLength;
      flowlapseArcLengthLutNormalized[i] = encodeFlowlapseNormalizedValue(normalizedDistance);
    }
  }

  flowlapseArcLengthLutValid = true;
}

float mapFlowlapseDistanceFractionToCurveT(uint8_t segmentBaseIndex, float distanceFraction) {
  float clampedDistanceFraction = clampFlowlapse01(distanceFraction);
  if (!FLOWLAPSE_ARC_LENGTH_SAMPLING_ENABLED) {
    return clampedDistanceFraction;
  }

  if (!flowlapseArcLengthLutValid || flowlapseArcLengthLutSegmentBaseIndex != segmentBaseIndex) {
    buildFlowlapseArcLengthLut(segmentBaseIndex);
  }

  for (uint8_t i = 1; i <= FLOWLAPSE_ARC_LENGTH_TABLE_STEPS; ++i) {
    float segmentEndDistance = decodeFlowlapseNormalizedValue(flowlapseArcLengthLutNormalized[i]);
    if (clampedDistanceFraction <= segmentEndDistance) {
      float segmentStartDistance = decodeFlowlapseNormalizedValue(flowlapseArcLengthLutNormalized[i - 1]);
      float segmentSpanDistance = segmentEndDistance - segmentStartDistance;
      float localFraction = 0.0f;
      if (segmentSpanDistance > 0.00001f) {
        localFraction = (clampedDistanceFraction - segmentStartDistance) / segmentSpanDistance;
      }

      float segmentStartT = static_cast<float>(i - 1) / static_cast<float>(FLOWLAPSE_ARC_LENGTH_TABLE_STEPS);
      float segmentEndT = static_cast<float>(i) / static_cast<float>(FLOWLAPSE_ARC_LENGTH_TABLE_STEPS);
      return segmentStartT + (segmentEndT - segmentStartT) * localFraction;
    }
  }

  return 1.0f;
}

float computeFlowlapseSegmentLength(uint8_t segmentBaseIndex, bool useCurvedPath) {
  if (!useCurvedPath) {
    return computeFlowlapseWaypointDistance(flowlapseWaypoints[segmentBaseIndex], flowlapseWaypoints[segmentBaseIndex + 1]);
  }

  FlowlapseWaypoint previousSample = interpolateFlowlapseCurvedPoint(segmentBaseIndex, 0.0f);
  float accumulatedLength = 0.0f;
  for (uint8_t i = 1; i <= FLOWLAPSE_ARC_LENGTH_TABLE_STEPS; ++i) {
    float sampleT = static_cast<float>(i) / static_cast<float>(FLOWLAPSE_ARC_LENGTH_TABLE_STEPS);
    FlowlapseWaypoint currentSample = interpolateFlowlapseCurvedPoint(segmentBaseIndex, sampleT);
    accumulatedLength += computeFlowlapseWaypointDistance(previousSample, currentSample);
    previousSample = currentSample;
  }

  return accumulatedLength;
}

void rebuildFlowlapsePathLengthCache(bool useCurvedPath) {
  flowlapsePathTotalLengthStored = 0;
  for (uint8_t segmentIndex = 0; segmentIndex < FLOWLAPSE_MAX_WAYPOINTS - 1; ++segmentIndex) {
    flowlapsePathSegmentCumulative[segmentIndex] = 0;
  }

  if (flowlapseWaypointCount < 2) {
    return;
  }

  float runningTotalLength = 0.0f;
  uint8_t segmentCount = static_cast<uint8_t>(flowlapseWaypointCount - 1);
  for (uint8_t segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex) {
    float segmentLength = computeFlowlapseSegmentLength(segmentIndex, useCurvedPath);
    runningTotalLength += segmentLength;
    flowlapsePathSegmentCumulative[segmentIndex] = encodeFlowlapseStoredLength(runningTotalLength);
  }
  flowlapsePathTotalLengthStored = encodeFlowlapseStoredLength(runningTotalLength);
}

FlowlapseWaypoint sampleFlowlapsePointAtPathDistance(float pathDistance, bool useCurvedPath) {
  float pathTotalLength = getFlowlapsePathTotalLength();
  if (flowlapseWaypointCount < 2 || pathTotalLength <= 0.0001f) {
    return flowlapseWaypoints[0];
  }

  float clampedDistance = pathDistance;
  if (clampedDistance < 0.0f) {
    clampedDistance = 0.0f;
  }
  if (clampedDistance > pathTotalLength) {
    clampedDistance = pathTotalLength;
  }

  uint8_t segmentCount = static_cast<uint8_t>(flowlapseWaypointCount - 1);
  uint8_t segmentIndex = 0;
  while (segmentIndex < segmentCount - 1
      && clampedDistance > decodeFlowlapseStoredLength(flowlapsePathSegmentCumulative[segmentIndex])) {
    segmentIndex++;
  }

  float segmentStartDistance = (segmentIndex == 0)
      ? 0.0f
      : decodeFlowlapseStoredLength(flowlapsePathSegmentCumulative[segmentIndex - 1]);
  float segmentLength = getFlowlapsePathSegmentLength(segmentIndex);
  float segmentDistance = clampedDistance - segmentStartDistance;
  float segmentFraction = 0.0f;
  if (segmentLength > 0.0001f) {
    segmentFraction = segmentDistance / segmentLength;
  }
  segmentFraction = clampFlowlapse01(segmentFraction);

  if (useCurvedPath) {
    float curveT = mapFlowlapseDistanceFractionToCurveT(segmentIndex, segmentFraction);
    return interpolateFlowlapseCurvedPoint(segmentIndex, curveT);
  }

  return interpolateFlowlapseLinearPointBetween(segmentIndex, static_cast<uint8_t>(segmentIndex + 1), segmentFraction);
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
  flowlapseJogIndex = 0;
  flowlapseL3HoldActive = false;
  flowlapseL3HoldStartMs = 0;
  suppressDroneNextL3Release = false;
  flowlapseState = FLOWLAPSE_STATE_RECORDING;
  flowlapseCapturePhase = FLOWLAPSE_CAPTURE_TRIGGER_LOW;
  flowlapseCapturePhaseStartMs = 0;
  flowlapseCaptureDwellMs = 0;
  flowlapsePreviewHoldUntilMs = 0;
  flowlapseTargetWaypointIndex = 0;
  flowlapseCaptureAlignedToFirstWaypoint = false;
  flowlapseCaptureDirection = 1;
  invalidateFlowlapseArcLengthLut();
  flowlapseFrameCountModeActive = false;
  flowlapseFrameCountTarget = 0;
  flowlapseFrameCountCurrentStopIndex = 0;
  flowlapseFrameCountSpacingStored = 0;
  flowlapsePreviewFrameModeActive = false;
  flowlapsePreviewFrameTarget = 0;
  flowlapsePreviewFrameStopIndex = 0;
  flowlapsePathTotalLengthStored = 0;
  flowlapseLastUpdateMs = millis();
  flowlapseLastProgressLogMs = 0;
  flowlapseCaptureRunStartMs = 0;
  flowlapsePauseStartMs = 0;
  flowlapseTotalPausedMs = 0;
  flowlapsePingPongEdgeCount = 0;
  resetFlowlapseAxisTierState(flowlapseLastUpdateMs);
  digitalWrite(trigger, HIGH);

  if (resetEstimatedPosition) {
    flowlapseCurrentSwingPos = 0.0f;
    flowlapseCurrentLiftPos = 0.0f;
    flowlapseCurrentPanPos = 0.0f;
    flowlapseCurrentTiltPos = 0.0f;
  }

  broadcastFlowlapseWaypointCount();
}

const char* getFlowlapseCapturePhaseLabel(FlowlapseCapturePhase phase) {
  switch (phase) {
    case FLOWLAPSE_CAPTURE_TRIGGER_LOW:
      return "trigger-low";
    case FLOWLAPSE_CAPTURE_TRIGGER_HIGH:
      return "trigger-high";
    case FLOWLAPSE_CAPTURE_MOVE_ACTIVE:
      return "move";
    case FLOWLAPSE_CAPTURE_DWELL:
      return "dwell";
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

  unsigned long elapsedSeconds = (flowlapseCaptureRunStartMs > 0 && now >= flowlapseCaptureRunStartMs)
      ? ((now - flowlapseCaptureRunStartMs - flowlapseTotalPausedMs) / 1000UL)
      : 0UL;
  unsigned int pingPongCycles = static_cast<unsigned int>(flowlapsePingPongEdgeCount / 2U);

  if (flowlapseFrameCountModeActive) {
    uint16_t totalFrames = flowlapseFrameCountTarget;
    uint16_t completedFrames = static_cast<uint16_t>(flowlapseFrameCountCurrentStopIndex + 1);
    if (completedFrames > totalFrames) {
      completedFrames = totalFrames;
    }

    uint8_t progressPercent = 0;
    if (totalFrames > 0) {
      progressPercent = static_cast<uint8_t>((static_cast<unsigned long>(completedFrames) * 100UL) / totalFrames);
    }

    unsigned long estimatedPerFrameMs = timelapseIntervalMs + static_cast<unsigned long>(stepDist) + flowlapseDwellMs;
    unsigned long remainingFrames = (totalFrames >= completedFrames) ? static_cast<unsigned long>(totalFrames - completedFrames) : 0;
    unsigned long etaSeconds = (remainingFrames * estimatedPerFrameMs) / 1000UL;

    char captureLine[160];
    snprintf(captureLine, sizeof(captureLine),
             "Flowlapse capture | frame %u/%u phase=%s progress=%u%% eta=%lus cycles=%u elapsed=%lus",
             static_cast<unsigned int>(completedFrames),
             static_cast<unsigned int>(totalFrames),
             getFlowlapseCapturePhaseLabel(flowlapseCapturePhase),
             static_cast<unsigned int>(progressPercent),
             etaSeconds,
             pingPongCycles,
             elapsedSeconds);
    Serial.println(captureLine);
    broadcastStatus(captureLine);
    return;
  }

  uint8_t totalSegments = (flowlapseWaypointCount > 1) ? static_cast<uint8_t>(flowlapseWaypointCount - 1) : 0;
  float pathPositionSegments = 0.0f;
  if (flowlapseCaptureAlignedToFirstWaypoint && totalSegments > 0) {
    if (flowlapseCaptureDirection >= 0) {
      if (flowlapseTargetWaypointIndex > 0) {
        pathPositionSegments = static_cast<float>(flowlapseTargetWaypointIndex - 1);
      }
    } else {
      if (flowlapseTargetWaypointIndex < flowlapseWaypointCount - 1) {
        pathPositionSegments = static_cast<float>(flowlapseTargetWaypointIndex + 1);
      } else {
        pathPositionSegments = static_cast<float>(totalSegments);
      }
    }

    if (pathPositionSegments > static_cast<float>(totalSegments)) {
      pathPositionSegments = static_cast<float>(totalSegments);
    }
  }

  float remainingMs = 0.0f;
  if (flowlapseCaptureAlignedToFirstWaypoint && totalSegments > 0) {
    unsigned long triggerLowMs = static_cast<unsigned long>(timelapseIntervalMs / 2);
    unsigned long triggerHighMs = static_cast<unsigned long>(timelapseIntervalMs / 2);
    unsigned long moveMs = static_cast<unsigned long>(stepDist);

    unsigned long dwellMsForSegment = flowlapseDwellMs;
    if (!flowlapseFrameCountModeActive && flowlapseTargetWaypointIndex < flowlapseWaypointCount) {
      dwellMsForSegment = flowlapseWaypoints[flowlapseTargetWaypointIndex].dwellMs;
    }

    unsigned long segmentTotalMs = triggerLowMs + triggerHighMs + moveMs + dwellMsForSegment;
    unsigned long phaseElapsedMs = now - flowlapseCapturePhaseStartMs;
    unsigned long segmentElapsedMs = 0;

    switch (flowlapseCapturePhase) {
      case FLOWLAPSE_CAPTURE_TRIGGER_LOW:
        segmentElapsedMs = phaseElapsedMs;
        break;
      case FLOWLAPSE_CAPTURE_TRIGGER_HIGH:
        segmentElapsedMs = triggerLowMs + phaseElapsedMs;
        break;
      case FLOWLAPSE_CAPTURE_MOVE_ACTIVE:
        segmentElapsedMs = triggerLowMs + triggerHighMs + phaseElapsedMs;
        break;
      case FLOWLAPSE_CAPTURE_DWELL:
        segmentElapsedMs = triggerLowMs + triggerHighMs + moveMs + phaseElapsedMs;
        break;
      default:
        segmentElapsedMs = 0;
        break;
    }

    if (segmentElapsedMs > segmentTotalMs) {
      segmentElapsedMs = segmentTotalMs;
    }

    float segmentFraction = 0.0f;
    if (segmentTotalMs > 0) {
      segmentFraction = static_cast<float>(segmentElapsedMs) / static_cast<float>(segmentTotalMs);
    }

    if (flowlapseCaptureDirection >= 0) {
      pathPositionSegments += segmentFraction;
    } else {
      pathPositionSegments -= segmentFraction;
    }

    if (pathPositionSegments < 0.0f) {
      pathPositionSegments = 0.0f;
    }
    if (pathPositionSegments > static_cast<float>(totalSegments)) {
      pathPositionSegments = static_cast<float>(totalSegments);
    }

    remainingMs = static_cast<float>(segmentTotalMs > segmentElapsedMs ? (segmentTotalMs - segmentElapsedMs) : 0UL);

    if (!flowlapseFrameCountModeActive && flowlapseCaptureDirection >= 0 && flowlapseTargetWaypointIndex < flowlapseWaypointCount) {
      for (uint8_t segmentEndIdx = static_cast<uint8_t>(flowlapseTargetWaypointIndex + 1);
           segmentEndIdx < flowlapseWaypointCount;
           ++segmentEndIdx) {
        unsigned long futureDwellMs = flowlapseWaypoints[segmentEndIdx].dwellMs;
        remainingMs += static_cast<float>(timelapseIntervalMs + static_cast<unsigned long>(stepDist) + futureDwellMs);
      }
    } else if (!flowlapseFrameCountModeActive && flowlapseCaptureDirection < 0) {
      float reverseRemainingSegments = pathPositionSegments;
      if (reverseRemainingSegments < 0.0f) {
        reverseRemainingSegments = 0.0f;
      }
      remainingMs = reverseRemainingSegments * static_cast<float>(timelapseIntervalMs + static_cast<unsigned long>(stepDist) + flowlapseDwellMs);
    } else {
      float remainingSegmentsEstimate = static_cast<float>(totalSegments) - pathPositionSegments;
      if (remainingSegmentsEstimate < 0.0f) {
        remainingSegmentsEstimate = 0.0f;
      }
      remainingMs = remainingSegmentsEstimate * static_cast<float>(timelapseIntervalMs + static_cast<unsigned long>(stepDist) + flowlapseDwellMs);
    }
  }

  uint8_t progressPercent = 0;
  if (totalSegments > 0) {
    progressPercent = static_cast<uint8_t>((pathPositionSegments * 100.0f) / static_cast<float>(totalSegments));
  }

  unsigned long etaSeconds = static_cast<unsigned long>(remainingMs / 1000.0f);

  char captureLine[160];
  snprintf(captureLine, sizeof(captureLine),
           "Flowlapse capture | waypoint %u/%u phase=%s progress=%u%% eta=%lus cycles=%u elapsed=%lus",
           static_cast<unsigned int>(flowlapseTargetWaypointIndex + 1),
           static_cast<unsigned int>(flowlapseWaypointCount),
           getFlowlapseCapturePhaseLabel(flowlapseCapturePhase),
           static_cast<unsigned int>(progressPercent),
           etaSeconds,
           pingPongCycles,
           elapsedSeconds);
  Serial.println(captureLine);
  broadcastStatus(captureLine);
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
    Serial.println(F("Flowlapse: waypoint limit reached (8)."));
    return;
  }

  FlowlapseWaypoint candidateWaypoint;
  candidateWaypoint.swing = flowlapseCurrentSwingPos;
  candidateWaypoint.lift = flowlapseCurrentLiftPos;
  candidateWaypoint.pan = flowlapseCurrentPanPos;
  candidateWaypoint.tilt = flowlapseCurrentTiltPos;
  candidateWaypoint.dwellMs = static_cast<uint16_t>(flowlapseDwellMs);

  if (flowlapseWaypointCount > 0) {
    const FlowlapseWaypoint& previousWaypoint = flowlapseWaypoints[flowlapseWaypointCount - 1];
    float deltaSwing = candidateWaypoint.swing - previousWaypoint.swing;
    float deltaLift = candidateWaypoint.lift - previousWaypoint.lift;
    float deltaPan = candidateWaypoint.pan - previousWaypoint.pan;
    float deltaTilt = candidateWaypoint.tilt - previousWaypoint.tilt;
    float separation = sqrt(deltaSwing * deltaSwing + deltaLift * deltaLift + deltaPan * deltaPan + deltaTilt * deltaTilt);

    if (separation < FLOWLAPSE_MIN_WAYPOINT_SEPARATION) {
      startLockoutDeniedRumbleFeedback();
      Serial.println(F("Flowlapse: waypoint too close to previous point; move rig farther before recording."));
      return;
    }
  }

  FlowlapseWaypoint& waypoint = flowlapseWaypoints[flowlapseWaypointCount];
  waypoint = candidateWaypoint;
  flowlapseWaypointCount++;
  invalidateFlowlapseArcLengthLut();

  startFeedbackRumble(FLOWLAPSE_WAYPOINT_RUMBLE_PULSES, FLOWLAPSE_WAYPOINT_RUMBLE_ON_MS, FLOWLAPSE_WAYPOINT_RUMBLE_TOTAL_MS);
  
  char waypointLine[64];
  snprintf(waypointLine, sizeof(waypointLine), "Flowlapse: waypoint recorded %u/%u",
           static_cast<unsigned int>(flowlapseWaypointCount),
           static_cast<unsigned int>(FLOWLAPSE_MAX_WAYPOINTS));
  Serial.println(waypointLine);
  emitControlIndicator("L3_WAYPOINT_RECORD");
  broadcastStatus(waypointLine);
  broadcastFlowlapseWaypointCount();
  
  if (DRONE_SERIAL_LOG_ENABLED) {
    Serial.print(F("  swing="));
    Serial.print(waypoint.swing, 2);
    Serial.print(F(" lift="));
    Serial.print(waypoint.lift, 2);
    Serial.print(F(" pan="));
    Serial.print(waypoint.pan, 2);
    Serial.print(F(" tilt="));
    Serial.println(waypoint.tilt, 2);
  }
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
  if (absError > FLOWLAPSE_AXIS_STOP_TOLERANCE) {
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
    Serial.println(F("Flowlapse: need at least 2 waypoints for preview."));
    return;
  }

  bool useCurvedPath = FLOWLAPSE_CURVED_PATH_ENABLED && flowlapseWaypointCount >= 3;
  flowlapseState = FLOWLAPSE_STATE_PREVIEW_RUNNING;
  flowlapseTargetWaypointIndex = 0;
  flowlapsePreviewHoldUntilMs = 0;
  flowlapsePreviewFrameModeActive = flowlapseFrameCountModeEnabled;
  flowlapsePreviewFrameTarget = FLOWLAPSE_FRAME_COUNT_TARGET;
  if (flowlapsePreviewFrameTarget < 2) {
    flowlapsePreviewFrameTarget = 2;
  }
  flowlapsePreviewFrameStopIndex = 0;

  if (flowlapsePreviewFrameModeActive) {
    rebuildFlowlapsePathLengthCache(useCurvedPath);
    Serial.print(F("Flowlapse: frame-count preview started. frames="));
    Serial.print(flowlapsePreviewFrameTarget);
    Serial.print(F(" spacing="));
    float pathTotalLength = getFlowlapsePathTotalLength();
    float previewSpacing = (flowlapsePreviewFrameTarget > 1)
        ? (pathTotalLength / static_cast<float>(flowlapsePreviewFrameTarget - 1))
        : 0.0f;
    Serial.println(previewSpacing, 1);
  } else {
    Serial.println(F("Flowlapse: preview started."));
    broadcastStatus("Flowlapse: preview started.");
  }

  resetFlowlapseAxisTierState(millis());
}

void startFlowlapseCapture(unsigned long now) {
  if (flowlapseWaypointCount < 2) {
    startLockoutDeniedRumbleFeedback();
    Serial.println(F("Flowlapse: need at least 2 waypoints to run capture."));
    return;
  }

  flowlapseState = FLOWLAPSE_STATE_CAPTURE_RUNNING;
  flowlapseTargetWaypointIndex = 0;
  flowlapseCaptureAlignedToFirstWaypoint = false;
  flowlapseCaptureDirection = 1;
  flowlapseCapturePhase = FLOWLAPSE_CAPTURE_TRIGGER_LOW;
  flowlapseCapturePhaseStartMs = now;
  flowlapseCaptureDwellMs = 0;
  flowlapseLastProgressLogMs = 0;
  flowlapseCaptureRunStartMs = now;
  flowlapsePauseStartMs = 0;
  flowlapseTotalPausedMs = 0;
  flowlapsePingPongEdgeCount = 0;
  resetFlowlapseAxisTierState(now);

  bool useCurvedPath = FLOWLAPSE_CURVED_PATH_ENABLED && flowlapseWaypointCount >= 3;
  flowlapseFrameCountModeActive = flowlapseFrameCountModeEnabled;
  flowlapseFrameCountTarget = FLOWLAPSE_FRAME_COUNT_TARGET;
  if (flowlapseFrameCountTarget < 2) {
    flowlapseFrameCountTarget = 2;
  }
  flowlapseFrameCountCurrentStopIndex = 0;
  flowlapseFrameCountSpacingStored = 0;
  rebuildFlowlapsePathLengthCache(useCurvedPath);
  if (flowlapseFrameCountModeActive) {
    float pathTotalLength = getFlowlapsePathTotalLength();
    if (flowlapseFrameCountTarget > 1) {
      flowlapseFrameCountSpacingStored = encodeFlowlapseStoredLength(
          pathTotalLength / static_cast<float>(flowlapseFrameCountTarget - 1));
    }
    Serial.print(F("Frame-count: "));
    Serial.print(flowlapseFrameCountTarget);
    Serial.print(F(" frames | spacing="));
    Serial.print(getFlowlapseFrameCountSpacing(), 1);
    Serial.println(F(" units"));
    Serial.print(F("Flowlapse: frame-count mode active. target="));
    Serial.print(flowlapseFrameCountTarget);
    Serial.print(F(" pathLength="));
    Serial.println(pathTotalLength, 2);
    if (FLOWLAPSE_LOOP_CAPTURE || flowlapsePingPongLoopEnabled) {
      Serial.println(F("Flowlapse: loop/ping-pong disabled while frame-count mode is active."));
    }
  }

  Serial.println(F("Flowlapse: capture run started."));
}

void enterDroneMode() {
  resetTimelapseState();
  resetBounceState();
  stopIntervalRumbleFeedback();
  normalizeMotionAxisSpeedStages(DEFAULT_AXIS_SPEED_STAGE);
  resetFlowlapseSession(true);
  droneMode = true;
  droneProportionalStickSpeedEnabled = false;
  droneManualSwingTier = DEFAULT_AXIS_SPEED_STAGE;
  droneManualLiftTier = DEFAULT_AXIS_SPEED_STAGE;
  droneManualPanTier = DEFAULT_AXIS_SPEED_STAGE;
  droneManualTiltTier = DEFAULT_AXIS_SPEED_STAGE;
  unsigned long now = millis();
  droneManualSwingTierLastStepMs = now;
  droneManualLiftTierLastStepMs = now;
  droneManualPanTierLastStepMs = now;
  droneManualTiltTierLastStepMs = now;
  droneLastActivityMs = now;
  Serial.println(F("DRONE MODE ACTIVATED - timelapse/bounce locked out"));
  broadcastStatus("MODE:DRONE");
  broadcastStatus("DRONE MODE ACTIVATED - timelapse/bounce locked out");
  Serial.println(F("Flowlapse: recording armed. L3=record waypoint, SELECT=stop record, L1+R1=wipe, L2+R2=undo last."));
  Serial.println(F("Flowlapse: START+SELECT+TRIANGLE toggles frame-count preview/capture mode."));
  Serial.println(F("Flowlapse: START+SELECT+CIRCLE toggles ping-pong loop mode."));
  Serial.println(F("Flowlapse: TRIANGLE cycles acceleration profile in ready states (linear/ease/cinematic/robotic)."));
  startDroneModeEnterRumbleFeedback();
}

void exitDroneMode() {
  droneMode = false;
  homeReturnActive = false;
  lastDronePrecisionModeActive = false;
  lastDroneBoostModeActive = false;
  droneAxisLastActiveFlags = 0;
  lastDroneSwingDirection = 0;
  lastDroneLiftDirection = 0;
  lastDronePanDirection = 0;
  lastDroneTiltDirection = 0;
  resetFlowlapseSession(true);
  droneManualSwingTier = DEFAULT_AXIS_SPEED_STAGE;
  droneManualLiftTier = DEFAULT_AXIS_SPEED_STAGE;
  droneManualPanTier = DEFAULT_AXIS_SPEED_STAGE;
  droneManualTiltTier = DEFAULT_AXIS_SPEED_STAGE;
  droneManualSwingTierLastStepMs = 0;
  droneManualLiftTierLastStepMs = 0;
  droneManualPanTierLastStepMs = 0;
  droneManualTiltTierLastStepMs = 0;
  droneLastActivityMs = 0;
  stopAllMotors();
  Serial.println(F("DRONE MODE DEACTIVATED"));
  broadcastStatus("MODE:MANUAL");
  broadcastStatus("DRONE MODE DEACTIVATED");
  startDroneModeExitRumbleFeedback();
}

void logDroneAxisStateIfChanged(bool current, uint8_t axisMask, const char* axisName) {
  bool last = isPackedStateSet(droneAxisLastActiveFlags, axisMask);
  if (current != last) {
    if (DRONE_SERIAL_LOG_ENABLED) {
      Serial.print(F("Drone axis | "));
      Serial.print(axisName);
      Serial.println(current ? " MOVING" : " STOPPED");
    }
    setPackedState(droneAxisLastActiveFlags, axisMask, current);
  }
}

void logDroneSwingStateIfChanged(bool current, int8_t direction) {
  bool lastActive = isPackedStateSet(droneAxisLastActiveFlags, DRONE_AXIS_LAST_SWING);
  int8_t currentDirection = current ? direction : 0;

  if (current != lastActive || currentDirection != lastDroneSwingDirection) {
    if (DRONE_SERIAL_LOG_ENABLED) {
      Serial.print(F("Drone axis | swing "));
      if (!current) {
        Serial.println(F("STOPPED"));
      } else if (currentDirection > 0) {
        Serial.println(F("MOVING RIGHT"));
      } else if (currentDirection < 0) {
        Serial.println(F("MOVING LEFT"));
      } else {
        Serial.println(F("MOVING"));
      }
    }

    setPackedState(droneAxisLastActiveFlags, DRONE_AXIS_LAST_SWING, current);
    lastDroneSwingDirection = currentDirection;
  }
}

void logDroneLiftStateIfChanged(bool current, int8_t direction) {
  bool lastActive = isPackedStateSet(droneAxisLastActiveFlags, DRONE_AXIS_LAST_LIFT);
  int8_t currentDirection = current ? direction : 0;

  if (current != lastActive || currentDirection != lastDroneLiftDirection) {
    if (DRONE_SERIAL_LOG_ENABLED) {
      Serial.print(F("Drone axis | lift "));
      if (!current) {
        Serial.println(F("STOPPED"));
      } else if (currentDirection > 0) {
        Serial.println(F("MOVING UP"));
      } else if (currentDirection < 0) {
        Serial.println(F("MOVING DOWN"));
      } else {
        Serial.println(F("MOVING"));
      }
    }

    setPackedState(droneAxisLastActiveFlags, DRONE_AXIS_LAST_LIFT, current);
    lastDroneLiftDirection = currentDirection;
  }
}

void logDronePanStateIfChanged(bool current, int8_t direction) {
  bool lastActive = isPackedStateSet(droneAxisLastActiveFlags, DRONE_AXIS_LAST_PAN);
  int8_t currentDirection = current ? direction : 0;

  if (current != lastActive || currentDirection != lastDronePanDirection) {
    if (DRONE_SERIAL_LOG_ENABLED) {
      Serial.print(F("Drone axis | pan "));
      if (!current) {
        Serial.println(F("STOPPED"));
      } else if (currentDirection > 0) {
        Serial.println(F("MOVING RIGHT"));
      } else if (currentDirection < 0) {
        Serial.println(F("MOVING LEFT"));
      } else {
        Serial.println(F("MOVING"));
      }
    }

    setPackedState(droneAxisLastActiveFlags, DRONE_AXIS_LAST_PAN, current);
    lastDronePanDirection = currentDirection;
  }
}

void logDroneTiltStateIfChanged(bool current, int8_t direction) {
  bool lastActive = isPackedStateSet(droneAxisLastActiveFlags, DRONE_AXIS_LAST_TILT);
  int8_t currentDirection = current ? direction : 0;

  if (current != lastActive || currentDirection != lastDroneTiltDirection) {
    if (DRONE_SERIAL_LOG_ENABLED) {
      Serial.print(F("Drone axis | tilt "));
      if (!current) {
        Serial.println(F("STOPPED"));
      } else if (currentDirection > 0) {
        Serial.println(F("MOVING UP"));
      } else if (currentDirection < 0) {
        Serial.println(F("MOVING DOWN"));
      } else {
        Serial.println(F("MOVING"));
      }
    }

    setPackedState(droneAxisLastActiveFlags, DRONE_AXIS_LAST_TILT, current);
    lastDroneTiltDirection = currentDirection;
  }
}

int8_t getDroneUiPercentFromCenteredDelta(int centeredDelta, int deadband) {
  if (abs(centeredDelta) <= deadband) {
    return 0;
  }

  int adjustedMagnitude = abs(centeredDelta) - deadband;
  int fullScale = DRONE_STICK_MAX_DEFLECTION - deadband;
  if (fullScale <= 0) {
    return 0;
  }

  int percent = (adjustedMagnitude * 100) / fullScale;
  if (percent > 100) {
    percent = 100;
  }

  return centeredDelta < 0 ? static_cast<int8_t>(-percent) : static_cast<int8_t>(percent);
}

void broadcastDroneUiStateIfDue(int8_t swingDirection, int8_t liftDirection, int8_t panDirection, int8_t tiltDirection,
                                bool leftStickClick, bool rightStickClick) {
  unsigned long now = millis();
  bool stateChanged = (swingDirection != lastBroadcastSwing ||
                       liftDirection  != lastBroadcastLift  ||
                       panDirection   != lastBroadcastPan   ||
                       tiltDirection  != lastBroadcastTilt ||
                       leftStickClick != lastBroadcastLeftStickClick ||
                       rightStickClick != lastBroadcastRightStickClick);

  // Send immediately on change; use interval only as keepalive when idle
  if (!stateChanged && (now - lastDroneUiBroadcastMs < DRONE_UI_BROADCAST_INTERVAL_MS)) {
    return;
  }

  char stateLine[64];
  snprintf(stateLine, sizeof(stateLine), "DRONE_STICK:%d,%d,%d,%d,%d,%d",
           static_cast<int>(swingDirection),
           static_cast<int>(liftDirection),
           static_cast<int>(panDirection),
           static_cast<int>(tiltDirection),
           leftStickClick ? 1 : 0,
           rightStickClick ? 1 : 0);
  sendToDisplayESP(stateLine);
  lastDroneUiBroadcastMs = now;
  lastBroadcastSwing = swingDirection;
  lastBroadcastLift  = liftDirection;
  lastBroadcastPan   = panDirection;
  lastBroadcastTilt  = tiltDirection;
  lastBroadcastLeftStickClick = leftStickClick;
  lastBroadcastRightStickClick = rightStickClick;
}

void logDroneSpeedModifierStateIfChanged() {
  bool precisionModeActive = DRONE_ENABLE_PRECISION_MODIFIER && ps2x.Button(PSB_L2);
  bool boostModeActive = DRONE_ENABLE_BOOST_MODIFIER
      && ps2x.Button(PSB_R2)
      && !(DRONE_L2_PRIORITY_OVER_BOOST && precisionModeActive);

  if (precisionModeActive != lastDronePrecisionModeActive || boostModeActive != lastDroneBoostModeActive) {
    char modifierLine[48];
    snprintf(modifierLine, sizeof(modifierLine), "DRONE_MODIFIER:precision=%u,boost=%u",
             precisionModeActive ? 1U : 0U,
             boostModeActive ? 1U : 0U);
    broadcastStatus(modifierLine);

    if (DRONE_SERIAL_LOG_ENABLED) {
      Serial.print(F("Drone speed modifier | precision="));
      Serial.print(precisionModeActive ? "ON" : "OFF");
      Serial.print(F(" boost="));
      Serial.println(boostModeActive ? "ON" : "OFF");
    }
    lastDronePrecisionModeActive = precisionModeActive;
    lastDroneBoostModeActive = boostModeActive;
  }
}

bool isFlowlapseBusyForHomeAction() {
  return (flowlapseState == FLOWLAPSE_STATE_PREVIEW_RUNNING
      || flowlapseState == FLOWLAPSE_STATE_CAPTURE_RUNNING
      || flowlapseState == FLOWLAPSE_STATE_CAPTURE_PAUSED
      || flowlapseState == FLOWLAPSE_STATE_UNDO_RUNNING
      || flowlapseState == FLOWLAPSE_STATE_JOG_RUNNING);
}

void setHomePoseFromCurrentPose(bool persistToEeprom) {
  homePose.swing = flowlapseCurrentSwingPos;
  homePose.lift = flowlapseCurrentLiftPos;
  homePose.pan = flowlapseCurrentPanPos;
  homePose.tilt = flowlapseCurrentTiltPos;
  homePose.dwellMs = 0;
  homePoseValid = true;

  if (persistToEeprom) {
    savePersistedSettings();
  }

  Serial.print(F("Home pose set | swing="));
  Serial.print(homePose.swing, 2);
  Serial.print(F(" lift="));
  Serial.print(homePose.lift, 2);
  Serial.print(F(" pan="));
  Serial.print(homePose.pan, 2);
  Serial.print(F(" tilt="));
  Serial.println(homePose.tilt, 2);
  broadcastStatus("HOME:SET");
}

void clearHomePose(bool persistToEeprom) {
  homePoseValid = false;
  homeReturnActive = false;
  homePose.swing = 0.0f;
  homePose.lift = 0.0f;
  homePose.pan = 0.0f;
  homePose.tilt = 0.0f;
  homePose.dwellMs = 0;

  if (persistToEeprom) {
    savePersistedSettings();
  }

  Serial.println(F("Home pose cleared."));
  broadcastStatus("HOME:CLEARED");
}

bool startHomeReturn(unsigned long now) {
  if (!homePoseValid) {
    startLockoutDeniedRumbleFeedback();
    Serial.println(F("Home return denied: no home pose is set."));
    broadcastStatus("HOME:NOT_SET");
    return false;
  }

  if (isFlowlapseBusyForHomeAction()) {
    startLockoutDeniedRumbleFeedback();
    Serial.println(F("Home return denied: flowlapse task is active."));
    broadcastStatus("HOME:BLOCKED");
    return false;
  }

  if (flowlapseState != FLOWLAPSE_STATE_RECORDING
      && flowlapseState != FLOWLAPSE_STATE_READY_FOR_PREVIEW
      && flowlapseState != FLOWLAPSE_STATE_READY_FOR_CAPTURE) {
    startLockoutDeniedRumbleFeedback();
    Serial.println(F("Home return denied: unsupported flowlapse state."));
    broadcastStatus("HOME:BLOCKED");
    return false;
  }

  stopAllMotors();
  resetFlowlapseAxisTierState(now);
  homeReturnActive = true;
  broadcastStatus("HOME:MOVING");
  Serial.println(F("Home return started."));
  return true;
}

void handleHomeReturnStep(unsigned long now, float deltaSeconds) {
  if (!homeReturnActive) {
    return;
  }

  if (!homePoseValid) {
    homeReturnActive = false;
    stopAllMotors();
    broadcastStatus("HOME:NOT_SET");
    return;
  }

  applyFlowlapseMotionTowardWaypoint(homePose, now, deltaSeconds);
  if (isFlowlapseTargetReached(homePose)) {
    homeReturnActive = false;
    stopAllMotors();
    startFeedbackRumble(1, FLOWLAPSE_WAYPOINT_RUMBLE_ON_MS, FLOWLAPSE_WAYPOINT_RUMBLE_TOTAL_MS);
    Serial.println(F("Home return complete."));
    broadcastStatus("HOME:REACHED");
  }
}

void handleSoloDirectionalMode(uint8_t buttonCode, bool isReversed, uint8_t normalPin, uint8_t reversedPin, uint8_t& modeState) {
  bool wasActive = (modeState == 1);
  bool isActive = ps2x.Button(PSB_SELECT) && ps2x.Button(buttonCode);

  if (DEBUG_EDGE_EVENTS && (buttonCode == PSB_PAD_LEFT || buttonCode == PSB_PAD_RIGHT)) {
    if (isActive && !wasActive) {
      Serial.print(F("SOLO press: "));
      Serial.println(buttonCode == PSB_PAD_LEFT ? "PAD_LEFT" : "PAD_RIGHT");
    } else if (!isActive && wasActive) {
      Serial.print(F("SOLO release: "));
      Serial.println(buttonCode == PSB_PAD_LEFT ? "PAD_LEFT" : "PAD_RIGHT");
    }
  }

  if (isActive) {
    setDirectionalOutput(isReversed, normalPin, reversedPin, HIGH);
    modeState = 1;
  } else {
    digitalWrite(normalPin, LOW);
    digitalWrite(reversedPin, LOW);
    modeState = 0;
  }
}

void handleCombinedDirectionalMode(uint8_t buttonCode, bool axis1Reversed, uint8_t axis1Normal, uint8_t axis1Rev,
                                    bool axis2Reversed, uint8_t axis2Normal, uint8_t axis2Rev,
                                    uint8_t& soloState, uint8_t motionFlagMask) {
  bool wasActive = isPackedStateSet(motionFlags, motionFlagMask);
  bool isActive = (soloState == 0 && ps2x.Button(buttonCode));

  if (DEBUG_EDGE_EVENTS && (buttonCode == PSB_PAD_LEFT || buttonCode == PSB_PAD_RIGHT)) {
    if (isActive && !wasActive) {
      Serial.print(F("COMBINED press: "));
      Serial.println(buttonCode == PSB_PAD_LEFT ? "PAD_LEFT" : "PAD_RIGHT");
    } else if (!isActive && wasActive) {
      Serial.print(F("COMBINED release: "));
      Serial.println(buttonCode == PSB_PAD_LEFT ? "PAD_LEFT" : "PAD_RIGHT");
    }
  }

  setPackedState(motionFlags, motionFlagMask, isActive);

  if (isActive) {
    setDirectionalOutput(axis1Reversed, axis1Normal, axis1Rev, HIGH);
    setDirectionalOutput(axis2Reversed, axis2Normal, axis2Rev, HIGH);
  } else {
    digitalWrite(axis1Normal, LOW);
    digitalWrite(axis1Rev, LOW);
    digitalWrite(axis2Normal, LOW);
    digitalWrite(axis2Rev, LOW);
  }
}

void handleSwingOnly(uint8_t buttonCode, uint8_t swingNormalPin, uint8_t swingReversedPin) {
  handleSoloDirectionalMode(buttonCode, isSwingReversed, swingNormalPin, swingReversedPin, swingSoloMode);
}

void handleSwingAndPan(uint8_t buttonCode, uint8_t swingNormalPin, uint8_t swingReversedPin,
                        uint8_t panNormalPin, uint8_t panReversedPin) {
  handleCombinedDirectionalMode(buttonCode, isSwingReversed, swingNormalPin, swingReversedPin,
                                 isPanReversed, panNormalPin, panReversedPin, swingSoloMode, MOTION_FLAG_SWING);
}

void activatePanTrim(uint8_t pin, const char* label) {
  digitalWrite(pin, HIGH);
  Serial.println(label);
  panStop = PAN_STOP_TRIM;
}

void applyPanTrimDuringSwing() {
  if (swingInMotion) {
    if (rightStickXvalue <= STICK_DEADBAND) {
      activatePanTrim(panSpeedDownOnly, "panSpeedDownOnly");
    } else if (rightStickXvalue >= STICK_MAX - STICK_DEADBAND) {
      activatePanTrim(panSpeedUpOnly, "panSpeedUpOnly");
    }
  }
}

void resetPanTrimAtCenter() {
  if (panStop == PAN_STOP_TRIM && abs(rightStickXvalue - STICK_CENTER) <= STICK_DEADBAND) {
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
  if (!swingInMotion && panStop == PAN_STOP_ACTIVE && abs(rightStickXvalue - STICK_CENTER) <= STICK_DEADBAND) {
    digitalWrite(panLeft, LOW);
    digitalWrite(panRight, LOW);
    digitalWrite(panSpeedUpOnly, LOW);
    digitalWrite(panSpeedDownOnly, LOW);
    panStop = PAN_STOP_NONE;
  }
}

void handlePanAxis() {
  int8_t panSoloDirection = 0;

  if (!swingInMotion) {
    if (rightStickXvalue <= STICK_DEADBAND) {
      panSoloDirection = -1;
      activatePanOnlyMotion(panLeft, panRight, "pan left only with top speed");
    } else if (rightStickXvalue >= STICK_MAX - STICK_DEADBAND) {
      panSoloDirection = 1;
      activatePanOnlyMotion(panRight, panLeft, "pan right only with top speed");
    }
  }

  if (panSoloDirection != lastPanSoloDirection) {
    if (panSoloDirection < 0) {
      emitControlIndicator("PAN_SOLO_LEFT");
    } else if (panSoloDirection > 0) {
      emitControlIndicator("PAN_SOLO_RIGHT");
    }
    lastPanSoloDirection = panSoloDirection;
  }

  stopPanOnlyMotionAtCenter();
}

void handleLiftOnly(uint8_t buttonCode, uint8_t liftNormalPin, uint8_t liftReversedPin) {
  handleSoloDirectionalMode(buttonCode, isLiftReversed, liftNormalPin, liftReversedPin, liftSoloMode);
}

void handleTiltTrimAxis() {
  if (liftInMotion && rightStickYvalue >= STICK_MAX - STICK_DEADBAND) {
    Serial.println(F("tiltSpeedDownOnly"));
    digitalWrite(tiltSpeedDownOnly, HIGH);
    tiltStop = TILT_STOP_TRIM;
  }
  if (liftInMotion && rightStickYvalue <= STICK_DEADBAND) {
    Serial.println(F("tiltSpeedUpOnly"));
    digitalWrite(tiltSpeedUpOnly, HIGH);
    tiltStop = TILT_STOP_TRIM;
  }
  if (tiltStop == TILT_STOP_TRIM && abs(rightStickYvalue - STICK_CENTER) <= STICK_DEADBAND) {
    digitalWrite(tiltSpeedUpOnly, LOW);
    digitalWrite(tiltSpeedDownOnly, LOW);
    tiltStop = TILT_STOP_NONE;
  }
}

void handleLiftAndTilt(uint8_t buttonCode, uint8_t liftNormalPin, uint8_t liftReversedPin,
                       uint8_t tiltNormalPin, uint8_t tiltReversedPin) {
  handleCombinedDirectionalMode(buttonCode, isLiftReversed, liftNormalPin, liftReversedPin,
                                isTiltReversed, tiltNormalPin, tiltReversedPin, liftSoloMode, MOTION_FLAG_LIFT);
}

void handleSwingMotionGroup() {
  bool soloLeftActive = ps2x.Button(PSB_SELECT) && ps2x.Button(PSB_PAD_LEFT);
  bool soloRightActive = ps2x.Button(PSB_SELECT) && ps2x.Button(PSB_PAD_RIGHT);

  if (soloLeftActive) {
    if (!lastSwingSoloLeftActive) {
      emitControlIndicator("SWING_SOLO_LEFT");
    }
    lastSwingSoloLeftActive = true;
    lastSwingSoloRightActive = false;
    setDirectionalOutput(isSwingReversed, swingLeft, swingRight, HIGH);
    swingSoloMode = 1;
    setPackedState(motionFlags, MOTION_FLAG_SWING, false);
    digitalWrite(panLeft, LOW);
    digitalWrite(panRight, LOW);
    return;
  }

  if (soloRightActive) {
    if (!lastSwingSoloRightActive) {
      emitControlIndicator("SWING_SOLO_RIGHT");
    }
    lastSwingSoloRightActive = true;
    lastSwingSoloLeftActive = false;
    setDirectionalOutput(isSwingReversed, swingRight, swingLeft, HIGH);
    swingSoloMode = 1;
    setPackedState(motionFlags, MOTION_FLAG_SWING, false);
    digitalWrite(panLeft, LOW);
    digitalWrite(panRight, LOW);
    return;
  }

  lastSwingSoloLeftActive = false;
  lastSwingSoloRightActive = false;

  if (swingSoloMode != 0) {
    digitalWrite(swingLeft, LOW);
    digitalWrite(swingRight, LOW);
    swingSoloMode = 0;
  }

  if (ps2x.Button(PSB_PAD_LEFT)) {
    if (ps2x.ButtonPressed(PSB_PAD_LEFT) && !ps2x.Button(PSB_SELECT)) {
      emitControlIndicator("SWING_PAN_LEFT");
    }
    setPackedState(motionFlags, MOTION_FLAG_SWING, true);
    setDirectionalOutput(isSwingReversed, swingLeft, swingRight, HIGH);
    setDirectionalOutput(isPanReversed, panRight, panLeft, HIGH);
  } else if (ps2x.Button(PSB_PAD_RIGHT)) {
    if (ps2x.ButtonPressed(PSB_PAD_RIGHT) && !ps2x.Button(PSB_SELECT)) {
      emitControlIndicator("SWING_PAN_RIGHT");
    }
    setPackedState(motionFlags, MOTION_FLAG_SWING, true);
    setDirectionalOutput(isSwingReversed, swingRight, swingLeft, HIGH);
    setDirectionalOutput(isPanReversed, panLeft, panRight, HIGH);
  } else {
    if (isPackedStateSet(motionFlags, MOTION_FLAG_SWING)) {
      digitalWrite(swingLeft, LOW);
      digitalWrite(swingRight, LOW);
      digitalWrite(panLeft, LOW);
      digitalWrite(panRight, LOW);
      setPackedState(motionFlags, MOTION_FLAG_SWING, false);
    }
  }
}

void handleLiftMotionGroup() {
  bool soloUpActive = ps2x.Button(PSB_SELECT) && ps2x.Button(PSB_PAD_UP);
  bool soloDownActive = ps2x.Button(PSB_SELECT) && ps2x.Button(PSB_PAD_DOWN);

  if (soloUpActive) {
    if (!lastLiftSoloUpActive) {
      emitControlIndicator("LIFT_SOLO_UP");
    }
    lastLiftSoloUpActive = true;
    lastLiftSoloDownActive = false;
    setDirectionalOutput(isLiftReversed, liftUp, liftDown, HIGH);
    liftSoloMode = 1;
    setPackedState(motionFlags, MOTION_FLAG_LIFT, false);
    digitalWrite(tiltUp, LOW);
    digitalWrite(tiltDown, LOW);
    return;
  }

  if (soloDownActive) {
    if (!lastLiftSoloDownActive) {
      emitControlIndicator("LIFT_SOLO_DOWN");
    }
    lastLiftSoloDownActive = true;
    lastLiftSoloUpActive = false;
    setDirectionalOutput(isLiftReversed, liftDown, liftUp, HIGH);
    liftSoloMode = 1;
    setPackedState(motionFlags, MOTION_FLAG_LIFT, false);
    digitalWrite(tiltUp, LOW);
    digitalWrite(tiltDown, LOW);
    return;
  }

  lastLiftSoloUpActive = false;
  lastLiftSoloDownActive = false;

  if (liftSoloMode != 0) {
    digitalWrite(liftUp, LOW);
    digitalWrite(liftDown, LOW);
    liftSoloMode = 0;
  }

  if (ps2x.Button(PSB_PAD_UP)) {
    if (ps2x.ButtonPressed(PSB_PAD_UP) && !ps2x.Button(PSB_SELECT)) {
      emitControlIndicator("LIFT_TILT_UP");
    }
    setPackedState(motionFlags, MOTION_FLAG_LIFT, true);
    setDirectionalOutput(isLiftReversed, liftUp, liftDown, HIGH);
    setDirectionalOutput(isTiltReversed, tiltDown, tiltUp, HIGH);
  } else if (ps2x.Button(PSB_PAD_DOWN)) {
    if (ps2x.ButtonPressed(PSB_PAD_DOWN) && !ps2x.Button(PSB_SELECT)) {
      emitControlIndicator("LIFT_TILT_DOWN");
    }
    setPackedState(motionFlags, MOTION_FLAG_LIFT, true);
    setDirectionalOutput(isLiftReversed, liftDown, liftUp, HIGH);
    setDirectionalOutput(isTiltReversed, tiltUp, tiltDown, HIGH);
  } else {
    if (isPackedStateSet(motionFlags, MOTION_FLAG_LIFT)) {
      digitalWrite(liftUp, LOW);
      digitalWrite(liftDown, LOW);
      digitalWrite(tiltUp, LOW);
      digitalWrite(tiltDown, LOW);
      setPackedState(motionFlags, MOTION_FLAG_LIFT, false);
    }
  }
}

void activateTiltOnlyMotion(uint8_t normalPin, uint8_t reversedPin, const char* label) {
  Serial.println(label);
  digitalWrite(tiltSpeedUpOnly, HIGH);
  digitalWrite(tiltSpeedDownOnly, HIGH);
  setDirectionalOutput(isTiltReversed, normalPin, reversedPin, HIGH);
  tiltStop = TILT_STOP_ACTIVE;
}

void stopTiltOnlyMotionAtCenter() {
  if (!liftInMotion && tiltStop == TILT_STOP_ACTIVE && abs(rightStickYvalue - STICK_CENTER) <= STICK_DEADBAND) {
    digitalWrite(tiltUp, LOW);
    digitalWrite(tiltDown, LOW);
    digitalWrite(tiltSpeedUpOnly, LOW);
    digitalWrite(tiltSpeedDownOnly, LOW);
    tiltStop = TILT_STOP_NONE;
  }
}

void handleTiltAxis() {
  int8_t tiltSoloDirection = 0;

  if (!liftInMotion) {
    if (rightStickYvalue >= STICK_MAX - STICK_DEADBAND) {
      tiltSoloDirection = -1;
      activateTiltOnlyMotion(tiltDown, tiltUp, "tiltDownOnly");
    } else if (rightStickYvalue <= STICK_DEADBAND) {
      tiltSoloDirection = 1;
      activateTiltOnlyMotion(tiltUp, tiltDown, "tiltUpOnly");
    }
  }

  if (tiltSoloDirection != lastTiltSoloDirection) {
    if (tiltSoloDirection < 0) {
      emitControlIndicator("TILT_SOLO_DOWN");
    } else if (tiltSoloDirection > 0) {
      emitControlIndicator("TILT_SOLO_UP");
    }
    lastTiltSoloDirection = tiltSoloDirection;
  }

  stopTiltOnlyMotionAtCenter();
}

void handleButtonDirection(uint8_t buttonCode, bool isReversed,
                           uint8_t normalPin, uint8_t reversedPin) {
  uint8_t outputState = ps2x.Button(buttonCode) ? HIGH : LOW;
  if (outputState == HIGH) {
    setDirectionalOutput(isReversed, normalPin, reversedPin, HIGH);
  } else {
    digitalWrite(normalPin, LOW);
    digitalWrite(reversedPin, LOW);
  }
}

void handleFocusAxis() {
  bool trianglePressed = ps2x.ButtonPressed(PSB_TRIANGLE);
  bool crossPressed = ps2x.ButtonPressed(PSB_CROSS);
  bool squarePressed = ps2x.ButtonPressed(PSB_SQUARE);
  bool circlePressed = ps2x.ButtonPressed(PSB_CIRCLE);

  if (ps2x.Button(PSB_TRIANGLE)) {
    setDirectionalOutput(isFocusReversed, focusLeft, focusRight, HIGH);
  }
  if (trianglePressed) {
    emitControlIndicator("FOCUS_LEFT");
  }
  if (ps2x.ButtonReleased(PSB_TRIANGLE)) {
    digitalWrite(focusLeft, LOW);
    digitalWrite(focusRight, LOW);
  }

  if (ps2x.Button(PSB_CROSS)) {
    setDirectionalOutput(isFocusReversed, focusRight, focusLeft, HIGH);
  }
  if (crossPressed) {
    emitControlIndicator("FOCUS_RIGHT");
  }
  if (ps2x.ButtonReleased(PSB_CROSS)) {
    digitalWrite(focusLeft, LOW);
    digitalWrite(focusRight, LOW);
  }

  if (WILL_TEST_LEGACY_HELD_SPEED_SIGNALS) {
    digitalWrite(focusSpeedDown, ps2x.Button(PSB_SQUARE) ? HIGH : LOW);
    if (squarePressed) {
      emitControlIndicator("FOCUS_SPEED_DOWN");
      emitSpeedTestEvent("FOCUS_SPEED_DOWN");
    }
  } else {
    if (squarePressed) {
      pulseSpeedStageUpPin(focusSpeedDown);
      emitControlIndicator("FOCUS_SPEED_DOWN");
      emitSpeedTestEvent("FOCUS_SPEED_DOWN");
    } else if (ps2x.ButtonReleased(PSB_SQUARE)) {
      digitalWrite(focusSpeedDown, LOW);
    }
  }

  if (WILL_TEST_LEGACY_HELD_SPEED_SIGNALS) {
    digitalWrite(focusSpeedUp, ps2x.Button(PSB_CIRCLE) ? HIGH : LOW);
    if (circlePressed) {
      emitControlIndicator("FOCUS_SPEED_UP");
      emitSpeedTestEvent("FOCUS_SPEED_UP");
    }
  } else {
    if (circlePressed) {
      pulseSpeedStageUpPin(focusSpeedUp);
      emitControlIndicator("FOCUS_SPEED_UP");
      emitSpeedTestEvent("FOCUS_SPEED_UP");
    } else if (ps2x.ButtonReleased(PSB_CIRCLE)) {
      digitalWrite(focusSpeedUp, LOW);
    }
  }
}

// Returns true if the axis is active (outside deadband), false if stopped.
bool applyDroneAxisControl(int stickValue, bool isReversed,
                           uint8_t negativeDirectionPin, uint8_t positiveDirectionPin,
                           uint8_t speedUpPin, uint8_t speedDownPin,
                           int axisDeadband, uint8_t maxSpeedTier, uint8_t expoPercent,
                           uint8_t& currentTier, unsigned long& lastStepMs,
                           unsigned long now) {
  digitalWrite(negativeDirectionPin, LOW);
  digitalWrite(positiveDirectionPin, LOW);

  uint8_t clampedMaxTier = static_cast<uint8_t>(
      constrain(static_cast<int>(maxSpeedTier), DRONE_SPEED_TIER_STOP, DRONE_SPEED_TIER_MAX));

  int signedOffsetFromCenter = stickValue - STICK_CENTER;
  if (abs(signedOffsetFromCenter) <= axisDeadband) {
    return false;
  }

  uint8_t targetTier = DRONE_SPEED_TIER_STOP;
  if (droneProportionalStickSpeedEnabled) {
    int magnitude = getStickDeflectionMagnitude(stickValue);
    targetTier = getProportionalTargetSpeedTier(magnitude, clampedMaxTier, expoPercent);
  } else {
    uint8_t fixedTier = static_cast<uint8_t>(constrain(
        static_cast<int>(DRONE_FIXED_STICK_SPEED_TIER),
        static_cast<int>(DRONE_SPEED_TIER_STOP),
        static_cast<int>(clampedMaxTier)));

    bool precisionModeActive = DRONE_ENABLE_PRECISION_MODIFIER && ps2x.Button(PSB_L2);
    bool boostModeActive = DRONE_ENABLE_BOOST_MODIFIER
      && ps2x.Button(PSB_R2)
      && !(DRONE_L2_PRIORITY_OVER_BOOST && precisionModeActive);

    if (precisionModeActive && fixedTier > DRONE_SPEED_TIER_STOP) {
      fixedTier--;
    } else if (boostModeActive && fixedTier < clampedMaxTier) {
      fixedTier = static_cast<uint8_t>(min(
          static_cast<int>(clampedMaxTier),
          static_cast<int>(fixedTier) + static_cast<int>(DRONE_BOOST_TIER_DELTA)));
    }

    targetTier = fixedTier;
  }

  if (DEBUG_DRONE_TIERS) {
    int magnitude = getStickDeflectionMagnitude(stickValue);
    Serial.print(F("  stick="));
    Serial.print(magnitude);
    Serial.print(F(" target="));
    Serial.print(targetTier);
    Serial.print(F(" current="));
    Serial.println(currentTier);
  }

  stepDroneAxisTierTowardTarget(currentTier, targetTier, speedUpPin, speedDownPin, lastStepMs, now);

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

  unsigned long now = millis();

  // Left stick controls swing (X) and lift (Y)
  bool swingActive = applyDroneAxisControl(leftStickXvalue, isSwingReversed, swingLeft, swingRight, swingSpeedUp, swingSpeedDown, DRONE_SWING_DEADBAND, DRONE_SWING_MAX_SPEED_TIER, DRONE_SWING_EXPO_PERCENT, droneManualSwingTier, droneManualSwingTierLastStepMs, now);
  bool liftActive  = applyDroneAxisControl(leftStickYvalue, isLiftReversed, liftUp, liftDown, liftSpeedUp, liftSpeedDown, DRONE_LIFT_DEADBAND, DRONE_LIFT_MAX_SPEED_TIER, DRONE_LIFT_EXPO_PERCENT, droneManualLiftTier, droneManualLiftTierLastStepMs, now);

  int8_t swingDirection = 0;
  if (leftStickXvalue < STICK_CENTER - DRONE_SWING_DEADBAND) {
    swingDirection = -1;
  } else if (leftStickXvalue > STICK_CENTER + DRONE_SWING_DEADBAND) {
    swingDirection = 1;
  }

  int8_t liftDirection = 0;
  if (leftStickYvalue < STICK_CENTER - DRONE_LIFT_DEADBAND) {
    liftDirection = 1;
  } else if (leftStickYvalue > STICK_CENTER + DRONE_LIFT_DEADBAND) {
    liftDirection = -1;
  }

  // Right stick controls pan (X) and tilt (Y)
  bool panActive  = applyDroneAxisControl(rightStickXvalue, isPanReversed, panLeft, panRight, panSpeedUp, panSpeedDown, DRONE_PAN_DEADBAND, DRONE_PAN_MAX_SPEED_TIER, DRONE_PAN_EXPO_PERCENT, droneManualPanTier, droneManualPanTierLastStepMs, now);
  bool tiltActive = applyDroneAxisControl(rightStickYvalue, isTiltReversed, tiltUp, tiltDown, tiltSpeedUp, tiltSpeedDown, DRONE_TILT_DEADBAND, DRONE_TILT_MAX_SPEED_TIER, DRONE_TILT_EXPO_PERCENT, droneManualTiltTier, droneManualTiltTierLastStepMs, now);

  int8_t panDirection = 0;
  if (rightStickXvalue < STICK_CENTER - DRONE_PAN_DEADBAND) {
    panDirection = -1;
  } else if (rightStickXvalue > STICK_CENTER + DRONE_PAN_DEADBAND) {
    panDirection = 1;
  }

  int8_t tiltDirection = 0;
  if (rightStickYvalue < STICK_CENTER - DRONE_TILT_DEADBAND) {
    tiltDirection = 1;
  } else if (rightStickYvalue > STICK_CENTER + DRONE_TILT_DEADBAND) {
    tiltDirection = -1;
  }

  int8_t swingUiPercent = getDroneUiPercentFromCenteredDelta(leftStickXvalue - STICK_CENTER, DRONE_UI_DEADBAND);
  int8_t liftUiPercent = getDroneUiPercentFromCenteredDelta(STICK_CENTER - leftStickYvalue, DRONE_UI_DEADBAND);
  int8_t panUiPercent = getDroneUiPercentFromCenteredDelta(rightStickXvalue - STICK_CENTER, DRONE_UI_DEADBAND);
  int8_t tiltUiPercent = getDroneUiPercentFromCenteredDelta(STICK_CENTER - rightStickYvalue, DRONE_UI_DEADBAND);
  bool leftStickClick = ps2x.Button(PSB_L3);
  bool rightStickClick = ps2x.Button(PSB_R3);

  logDroneSwingStateIfChanged(swingActive, swingDirection);
  logDroneLiftStateIfChanged(liftActive, liftDirection);
  logDronePanStateIfChanged(panActive, panDirection);
  logDroneTiltStateIfChanged(tiltActive, tiltDirection);
  broadcastDroneUiStateIfDue(swingUiPercent, liftUiPercent, panUiPercent, tiltUiPercent, leftStickClick, rightStickClick);

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
  flowlapsePreviewFrameModeActive = false;
  flowlapsePreviewFrameTarget = 0;
  flowlapsePreviewFrameStopIndex = 0;
  Serial.println(F("Flowlapse: preview complete. Press START to run capture."));
  broadcastStatus("Flowlapse: preview complete. Press START to run capture.");
}

void completeFlowlapseCapture(unsigned long now) {
  stopAllMotors();
  digitalWrite(trigger, HIGH);

  if (flowlapseFrameCountModeActive && FLOWLAPSE_FRAMECOUNT_AUTO_EXIT) {
    Serial.print(F("Frame-count capture complete ("));
    Serial.print(flowlapseFrameCountTarget);
    Serial.println(F(" frames) — mode exited"));
    startFrameCountCompleteRumbleFeedback();
    flowlapseFrameCountModeActive = false;
  } else if (flowlapseFrameCountModeActive) {
    Serial.print(F("Frame-count capture complete ("));
    Serial.print(flowlapseFrameCountTarget);
    Serial.println(F(" frames) — mode remains armed"));
  }

  if (!flowlapseFrameCountModeActive && flowlapsePingPongLoopEnabled && flowlapseWaypointCount >= 2) {
    flowlapsePingPongEdgeCount++;
    flowlapseCapturePhase = FLOWLAPSE_CAPTURE_TRIGGER_LOW;
    flowlapseCapturePhaseStartMs = now;
    flowlapseLastProgressLogMs = 0;
    resetFlowlapseAxisTierState(now);

    if (flowlapseCaptureDirection >= 0) {
      flowlapseCaptureDirection = -1;
      flowlapseTargetWaypointIndex = static_cast<uint8_t>(flowlapseWaypointCount - 2);
    } else {
      flowlapseCaptureDirection = 1;
      flowlapseTargetWaypointIndex = 1;
    }

    Serial.print(F("Flowlapse: ping-pong edge reached — reversing direction. cycles="));
    Serial.print(static_cast<unsigned int>(flowlapsePingPongEdgeCount / 2U));
    Serial.print(F(" elapsed="));
    Serial.print((flowlapseCaptureRunStartMs > 0 && now >= flowlapseCaptureRunStartMs)
        ? ((now - flowlapseCaptureRunStartMs - flowlapseTotalPausedMs) / 1000UL)
        : 0UL);
    Serial.println(F("s"));
  } else if (!flowlapseFrameCountModeActive && FLOWLAPSE_LOOP_CAPTURE) {
    flowlapseCaptureAlignedToFirstWaypoint = false;
    flowlapseCapturePhase = FLOWLAPSE_CAPTURE_TRIGGER_LOW;
    flowlapseCapturePhaseStartMs = now;
    flowlapseTargetWaypointIndex = 0;
    flowlapseCaptureDirection = 1;
    flowlapseLastProgressLogMs = 0;
    resetFlowlapseAxisTierState(now);
    Serial.println(F("Flowlapse: capture loop complete — restarting."));
  } else {
    bool shouldReturnToFirst = FLOWLAPSE_RETURN_TO_FIRST_WAYPOINT_ON_COMPLETE
                            && flowlapseWaypointCount > 0
                            && flowlapseTargetWaypointIndex != 0;
    const char* finalProgressMsg = "Flowlapse capture | progress=100% eta=0s";
    if (shouldReturnToFirst) {
      flowlapseState = FLOWLAPSE_STATE_JOG_RUNNING;
      flowlapseJogIndex = 0;
      resetFlowlapseAxisTierState(now);
      Serial.println(finalProgressMsg);
      broadcastStatus(finalProgressMsg);
      const char* completionMsg = "Flowlapse: capture complete. Returning to waypoint 1.";
      Serial.println(completionMsg);
      broadcastStatus(completionMsg);
      broadcastStatus("MODE:DRONE");
    } else {
      flowlapseState = FLOWLAPSE_STATE_RECORDING;
      flowlapseCapturePhase = FLOWLAPSE_CAPTURE_TRIGGER_LOW;
      flowlapseCapturePhaseStartMs = 0;
      flowlapseCaptureAlignedToFirstWaypoint = false;
      flowlapseTargetWaypointIndex = 0;
      flowlapseCaptureDirection = 1;
      Serial.println(finalProgressMsg);
      broadcastStatus(finalProgressMsg);
      const char* completionMsg = "Flowlapse: capture complete.";
      Serial.println(completionMsg);
      broadcastStatus(completionMsg);
      broadcastStatus("MODE:DRONE");
    }
  }
}

void handleFlowlapsePreviewStep(unsigned long now, float deltaSeconds) {
  if (flowlapsePreviewFrameModeActive) {
    if (flowlapsePreviewFrameStopIndex >= flowlapsePreviewFrameTarget) {
      completeFlowlapsePreview();
      return;
    }

    if (flowlapsePreviewHoldUntilMs != 0) {
      stopAllMotors();
      if (now < flowlapsePreviewHoldUntilMs) {
        return;
      }

      flowlapsePreviewHoldUntilMs = 0;
      flowlapsePreviewFrameStopIndex++;
      if (flowlapsePreviewFrameStopIndex >= flowlapsePreviewFrameTarget) {
        completeFlowlapsePreview();
        return;
      }
    }

    bool useCurvedPath = FLOWLAPSE_CURVED_PATH_ENABLED && flowlapseWaypointCount >= 3;
    float previewDenominator = static_cast<float>(flowlapsePreviewFrameTarget - 1);
    float previewFraction = (previewDenominator > 0.0f)
        ? (static_cast<float>(flowlapsePreviewFrameStopIndex) / previewDenominator)
        : 0.0f;
    float previewDistance = getFlowlapsePathTotalLength() * clampFlowlapse01(previewFraction);
    FlowlapseWaypoint previewTarget = sampleFlowlapsePointAtPathDistance(previewDistance, useCurvedPath);

    float previewSpeedMultiplier = ps2x.Button(PSB_L2) ? FLOWLAPSE_PREVIEW_SPEED_BOOST : FLOWLAPSE_PREVIEW_SPEED_SCALE;
    float scaledPreviewDeltaSeconds = deltaSeconds * previewSpeedMultiplier;
    applyFlowlapseMotionTowardWaypoint(previewTarget, now, scaledPreviewDeltaSeconds);

    if (isFlowlapseTargetReached(previewTarget)) {
      stopAllMotors();
      flowlapsePreviewHoldUntilMs = now + FLOWLAPSE_PREVIEW_POINT_HOLD_MS;
      Serial.print(F("Flowlapse preview reached frame stop "));
      Serial.print(static_cast<unsigned long>(flowlapsePreviewFrameStopIndex + 1));
      Serial.print(F("/"));
      Serial.println(flowlapsePreviewFrameTarget);
    }
    return;
  }

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
  float previewSpeedMultiplier = ps2x.Button(PSB_L2) ? FLOWLAPSE_PREVIEW_SPEED_BOOST : FLOWLAPSE_PREVIEW_SPEED_SCALE;
  float scaledPreviewDeltaSeconds = deltaSeconds * previewSpeedMultiplier;
  applyFlowlapseMotionTowardWaypoint(target, now, scaledPreviewDeltaSeconds);

  if (isFlowlapseTargetReached(target)) {
    stopAllMotors();
    flowlapsePreviewHoldUntilMs = now + FLOWLAPSE_PREVIEW_POINT_HOLD_MS;
    Serial.print(F("Flowlapse preview reached waypoint "));
    Serial.println(flowlapseTargetWaypointIndex + 1);
    char previewWaypointLine[48];
    snprintf(previewWaypointLine, sizeof(previewWaypointLine), "PREVIEW_WAYPOINT:%u/%u",
             static_cast<unsigned int>(flowlapseTargetWaypointIndex + 1),
             static_cast<unsigned int>(flowlapseWaypointCount));
    broadcastStatus(previewWaypointLine);
  }
}

void handleFlowlapseCaptureStep(unsigned long now, float deltaSeconds) {
  logFlowlapseCaptureProgressIfDue(now);

  if (flowlapseWaypointCount < 2) {
    completeFlowlapseCapture(now);
    return;
  }

  if (!flowlapseCaptureAlignedToFirstWaypoint) {
    const FlowlapseWaypoint& firstWaypoint = flowlapseWaypoints[0];
    applyFlowlapseMotionTowardWaypoint(firstWaypoint, now, deltaSeconds);

    if (isFlowlapseTargetReached(firstWaypoint)) {
      stopAllMotors();
      flowlapseCaptureAlignedToFirstWaypoint = true;
      flowlapseCaptureDirection = 1;
      flowlapseFrameCountCurrentStopIndex = 0;
      flowlapseCapturePhase = FLOWLAPSE_CAPTURE_TRIGGER_LOW;
      flowlapseCapturePhaseStartMs = now;
      flowlapseTargetWaypointIndex = 1;
      Serial.println(F("Flowlapse: aligned to first waypoint. Capture triggering started."));
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
        if (flowlapseFrameCountModeActive) {
          uint16_t nextStopIndex = static_cast<uint16_t>(flowlapseFrameCountCurrentStopIndex + 1);
          if (nextStopIndex >= flowlapseFrameCountTarget) {
            completeFlowlapseCapture(now);
            return;
          }
          flowlapseCapturePhase = FLOWLAPSE_CAPTURE_MOVE_ACTIVE;
          flowlapseCapturePhaseStartMs = now;
        } else {
          flowlapseCapturePhase = FLOWLAPSE_CAPTURE_MOVE_ACTIVE;
          flowlapseCapturePhaseStartMs = now;
        }
      }
      break;

    case FLOWLAPSE_CAPTURE_MOVE_ACTIVE:
      if (!flowlapseFrameCountModeActive && flowlapseTargetWaypointIndex >= flowlapseWaypointCount) {
        completeFlowlapseCapture(now);
        return;
      }

      unsigned long moveElapsedMs = now - flowlapseCapturePhaseStartMs;
      unsigned long moveDurationMs = static_cast<unsigned long>(stepDist);
      bool useCurvedPath = FLOWLAPSE_CURVED_PATH_ENABLED && flowlapseWaypointCount >= 3;

      float segmentT = (moveDurationMs > 0)
          ? static_cast<float>(moveElapsedMs) / static_cast<float>(moveDurationMs)
          : 1.0f;
      float easedSegmentT = applyFlowlapseEaseInOut(segmentT);

      FlowlapseWaypoint segmentTarget;
      if (flowlapseFrameCountModeActive) {
        uint16_t nextStopIndex = static_cast<uint16_t>(flowlapseFrameCountCurrentStopIndex + 1);
        float stopDenominator = static_cast<float>(flowlapseFrameCountTarget - 1);
        float nextStopFraction = (stopDenominator > 0.0f) ? (static_cast<float>(nextStopIndex) / stopDenominator) : 1.0f;
        float targetPathDistance = getFlowlapsePathTotalLength() * clampFlowlapse01(nextStopFraction);
        segmentTarget = sampleFlowlapsePointAtPathDistance(targetPathDistance, useCurvedPath);
      } else {
        uint8_t segmentStartIndex = (flowlapseCaptureDirection >= 0)
            ? static_cast<uint8_t>(flowlapseTargetWaypointIndex - 1)
            : static_cast<uint8_t>(flowlapseTargetWaypointIndex + 1);
        uint8_t segmentEndIndex = flowlapseTargetWaypointIndex;
        if (useCurvedPath) {
          uint8_t curveSegmentBaseIndex = (flowlapseCaptureDirection >= 0)
              ? segmentStartIndex
              : segmentEndIndex;
          float curveDistanceMappedT = mapFlowlapseDistanceFractionToCurveT(curveSegmentBaseIndex, easedSegmentT);
          float curveT = (flowlapseCaptureDirection >= 0)
            ? curveDistanceMappedT
            : (1.0f - curveDistanceMappedT);
          segmentTarget = interpolateFlowlapseCurvedPoint(curveSegmentBaseIndex, curveT);
        } else {
          segmentTarget = interpolateFlowlapseLinearPointBetween(segmentStartIndex, segmentEndIndex, easedSegmentT);
        }
      }
      applyFlowlapseMotionTowardWaypoint(segmentTarget, now, deltaSeconds);

      if (moveElapsedMs >= moveDurationMs) {
        unsigned long dwellForReachedStopMs = flowlapseDwellMs;

        flowlapseCurrentSwingPos = segmentTarget.swing;
        flowlapseCurrentLiftPos = segmentTarget.lift;
        flowlapseCurrentPanPos = segmentTarget.pan;
        flowlapseCurrentTiltPos = segmentTarget.tilt;

        if (flowlapseFrameCountModeActive) {
          flowlapseFrameCountCurrentStopIndex++;
        } else {
          uint8_t reachedWaypointIndex = flowlapseTargetWaypointIndex;
          if (reachedWaypointIndex < flowlapseWaypointCount) {
            dwellForReachedStopMs = flowlapseWaypoints[reachedWaypointIndex].dwellMs;
          }

          if (flowlapseCaptureDirection >= 0) {
            flowlapseTargetWaypointIndex++;
          } else {
            flowlapseTargetWaypointIndex--;
          }

          bool reachedForwardEnd = (flowlapseCaptureDirection >= 0) && (flowlapseTargetWaypointIndex >= flowlapseWaypointCount);
          bool reachedReverseEnd = (flowlapseCaptureDirection < 0) && (flowlapseTargetWaypointIndex == 255);
          if (reachedForwardEnd || reachedReverseEnd) {
            completeFlowlapseCapture(now);
            return;
          }
        }

        stopAllMotors();
        flowlapseCaptureDwellMs = dwellForReachedStopMs;
        if (flowlapseCaptureDwellMs > 0) {
          flowlapseCapturePhase = FLOWLAPSE_CAPTURE_DWELL;
        } else {
          flowlapseCapturePhase = FLOWLAPSE_CAPTURE_TRIGGER_LOW;
        }
        flowlapseCapturePhaseStartMs = now;
      }
      break;

    case FLOWLAPSE_CAPTURE_DWELL:
      stopAllMotors();
      if (now - flowlapseCapturePhaseStartMs >= flowlapseCaptureDwellMs) {
        flowlapseCapturePhase = FLOWLAPSE_CAPTURE_TRIGGER_LOW;
        flowlapseCapturePhaseStartMs = now;
      }
      break;
  }
}

void startFlowlapseUndoLastWaypoint(unsigned long now) {
  if (flowlapseWaypointCount == 0) {
    startLockoutDeniedRumbleFeedback();
    Serial.println(F("Flowlapse: no waypoint to undo."));
    return;
  }

  flowlapseState = FLOWLAPSE_STATE_UNDO_RUNNING;
  flowlapseTargetWaypointIndex = static_cast<uint8_t>(flowlapseWaypointCount - 1);
  resetFlowlapseAxisTierState(now);
  Serial.print(F("Flowlapse: undo moving to waypoint "));
  Serial.println(flowlapseTargetWaypointIndex + 1);
}

void handleFlowlapseUndoStep(unsigned long now, float deltaSeconds) {
  if (flowlapseWaypointCount == 0 || flowlapseTargetWaypointIndex >= flowlapseWaypointCount) {
    flowlapseState = FLOWLAPSE_STATE_RECORDING;
    stopAllMotors();
    return;
  }

  const FlowlapseWaypoint& target = flowlapseWaypoints[flowlapseTargetWaypointIndex];
  applyFlowlapseMotionTowardWaypoint(target, now, deltaSeconds);

  if (!isFlowlapseTargetReached(target)) {
    return;
  }

  stopAllMotors();
  flowlapseState = FLOWLAPSE_STATE_UNDO_READY_DELETE;
  Serial.print(F("Flowlapse: reached last waypoint "));
  Serial.print(flowlapseTargetWaypointIndex + 1);
  Serial.println(F(". Press L2+R2 again to delete it."));
}

void adjustFlowlapseDwell(long delta) {
  long newDwell = static_cast<long>(flowlapseDwellMs) + delta;
  if (newDwell < 0) newDwell = 0;
  if (newDwell > static_cast<long>(FLOWLAPSE_DWELL_MAX_MS)) newDwell = static_cast<long>(FLOWLAPSE_DWELL_MAX_MS);
  if (static_cast<unsigned long>(newDwell) == flowlapseDwellMs) {
    startLimitReachedRumbleFeedback();
    Serial.println(F("Flowlapse dwell limit reached."));
    return;
  }
  flowlapseDwellMs = static_cast<unsigned long>(newDwell);
  Serial.print(F("Flowlapse dwell (ms) = "));
  Serial.println(flowlapseDwellMs);
  if (flowlapseDwellMs == 0) {
    startFeedbackRumble(2, FLOWLAPSE_WAYPOINT_RUMBLE_ON_MS, FLOWLAPSE_WAYPOINT_RUMBLE_TOTAL_MS);
    Serial.println(F("Flowlapse dwell disabled (0 ms)."));
    return;
  }

  uint8_t dwellStepCount = static_cast<uint8_t>(constrain(
      static_cast<int>(flowlapseDwellMs / FLOWLAPSE_DWELL_ADJUST_INCREMENT_MS), 1, 20));
  startFeedbackRumble(dwellStepCount, FLOWLAPSE_WAYPOINT_RUMBLE_ON_MS, FLOWLAPSE_WAYPOINT_RUMBLE_TOTAL_MS);
}

void adjustFlowlapseWaypointDwell(uint8_t waypointIndex, long delta) {
  if (waypointIndex >= flowlapseWaypointCount) {
    startLockoutDeniedRumbleFeedback();
    Serial.println(F("Flowlapse: select a valid waypoint before editing dwell."));
    return;
  }

  long newDwell = static_cast<long>(flowlapseWaypoints[waypointIndex].dwellMs) + delta;
  if (newDwell < 0) newDwell = 0;
  if (newDwell > static_cast<long>(FLOWLAPSE_DWELL_MAX_MS)) newDwell = static_cast<long>(FLOWLAPSE_DWELL_MAX_MS);

  if (static_cast<uint16_t>(newDwell) == flowlapseWaypoints[waypointIndex].dwellMs) {
    startLimitReachedRumbleFeedback();
    Serial.println(F("Flowlapse: waypoint dwell limit reached."));
    return;
  }

  flowlapseWaypoints[waypointIndex].dwellMs = static_cast<uint16_t>(newDwell);
  Serial.print(F("Flowlapse: waypoint "));
  Serial.print(waypointIndex + 1);
  Serial.print(F(" dwell (ms) = "));
  Serial.println(flowlapseWaypoints[waypointIndex].dwellMs);

  if (flowlapseWaypoints[waypointIndex].dwellMs == 0) {
    startFeedbackRumble(2, FLOWLAPSE_WAYPOINT_RUMBLE_ON_MS, FLOWLAPSE_WAYPOINT_RUMBLE_TOTAL_MS);
    Serial.println(F("Flowlapse: waypoint dwell disabled (0 ms)."));
    return;
  }

  uint8_t dwellStepCount = static_cast<uint8_t>(constrain(
      static_cast<int>(flowlapseWaypoints[waypointIndex].dwellMs / FLOWLAPSE_DWELL_ADJUST_INCREMENT_MS), 1, 20));
  startFeedbackRumble(dwellStepCount, FLOWLAPSE_WAYPOINT_RUMBLE_ON_MS, FLOWLAPSE_WAYPOINT_RUMBLE_TOTAL_MS);
}

bool handleDroneFlowlapseButtons(unsigned long now) {
  static bool lastDroneManualShutterR1State = false;
  static bool lastDroneStickSpeedToggleComboActive = false;

  bool currentDroneManualShutterR1State = ps2x.Button(PSB_R1);
  bool r1Pressed = !lastDroneManualShutterR1State && currentDroneManualShutterR1State;
  lastDroneManualShutterR1State = currentDroneManualShutterR1State;

  bool manualShutterAllowed = flowlapseState != FLOWLAPSE_STATE_CAPTURE_RUNNING
      && flowlapseState != FLOWLAPSE_STATE_CAPTURE_PAUSED;

  bool manualShutterComboClear = !ps2x.Button(PSB_L1)
      && !ps2x.Button(PSB_START)
      && !ps2x.Button(PSB_SELECT);

  if (r1Pressed && manualShutterAllowed && manualShutterComboClear) {
    digitalWrite(trigger, LOW);
    delay(DRONE_MANUAL_SHUTTER_PULSE_MS);
    digitalWrite(trigger, HIGH);
    emitControlIndicator("DRONE_MANUAL_SHUTTER");
    broadcastStatus("TRIGGER:MANUAL_DRONE");
    droneLastActivityMs = now;
    return true;
  }

  bool loopModeToggleComboActive = ps2x.Button(PSB_START) && ps2x.Button(PSB_SELECT) && ps2x.Button(PSB_CIRCLE);
  if (loopModeToggleComboActive && !lastFlowlapseLoopToggleComboActive) {
    bool toggleAllowed = (flowlapseState != FLOWLAPSE_STATE_PREVIEW_RUNNING)
                      && (flowlapseState != FLOWLAPSE_STATE_CAPTURE_RUNNING)
                      && (flowlapseState != FLOWLAPSE_STATE_CAPTURE_PAUSED)
                      && (flowlapseState != FLOWLAPSE_STATE_UNDO_RUNNING)
                      && (flowlapseState != FLOWLAPSE_STATE_JOG_RUNNING);
    suppressDroneNextSelectRelease = true;
    suppressDroneNextStartRelease = true;

    if (!toggleAllowed) {
      startLockoutDeniedRumbleFeedback();
      Serial.println(F("Flowlapse: loop toggle blocked while preview/capture/undo/jog is running."));
    } else {
      flowlapsePingPongLoopEnabled = !flowlapsePingPongLoopEnabled;
      startFeedbackRumble(flowlapsePingPongLoopEnabled ? 2 : 1,
                          FLOWLAPSE_WAYPOINT_RUMBLE_ON_MS,
                          FLOWLAPSE_WAYPOINT_RUMBLE_TOTAL_MS);
      Serial.print(F("Flowlapse: ping-pong loop mode "));
      Serial.println(flowlapsePingPongLoopEnabled ? "enabled via controller toggle." : "disabled via controller toggle.");
      if (flowlapsePingPongLoopEnabled) {
        Serial.println(F("Flowlapse: loop path pattern = 1 -> 2 -> ... -> N -> ... -> 2 -> 1 -> repeat."));
      }
    }

    droneLastActivityMs = now;
  }
  lastFlowlapseLoopToggleComboActive = loopModeToggleComboActive;
  if (loopModeToggleComboActive) {
    stopAllMotors();
    return true;
  }

  bool frameModeToggleComboActive = ps2x.Button(PSB_START) && ps2x.Button(PSB_SELECT) && ps2x.Button(PSB_TRIANGLE);
  if (frameModeToggleComboActive && !lastFlowlapseFrameModeToggleComboActive) {
    bool toggleAllowed = (flowlapseState != FLOWLAPSE_STATE_PREVIEW_RUNNING)
                      && (flowlapseState != FLOWLAPSE_STATE_CAPTURE_RUNNING)
                      && (flowlapseState != FLOWLAPSE_STATE_CAPTURE_PAUSED)
                      && (flowlapseState != FLOWLAPSE_STATE_UNDO_RUNNING)
                      && (flowlapseState != FLOWLAPSE_STATE_JOG_RUNNING);
    suppressDroneNextSelectRelease = true;
    suppressDroneNextStartRelease = true;

    if (!toggleAllowed) {
      startLockoutDeniedRumbleFeedback();
      Serial.println(F("Flowlapse: frame-count toggle blocked while preview/capture/undo/jog is running."));
    } else {
      flowlapseFrameCountModeEnabled = !flowlapseFrameCountModeEnabled;
      startFeedbackRumble(flowlapseFrameCountModeEnabled ? 2 : 1,
                          FLOWLAPSE_WAYPOINT_RUMBLE_ON_MS,
                          FLOWLAPSE_WAYPOINT_RUMBLE_TOTAL_MS);
      Serial.print(F("Flowlapse: frame-count mode "));
      Serial.println(flowlapseFrameCountModeEnabled ? "enabled via controller toggle." : "disabled via controller toggle.");
      if (flowlapseFrameCountModeEnabled) {
        Serial.print(F("Flowlapse: target frames = "));
        Serial.println(FLOWLAPSE_FRAME_COUNT_TARGET);
      }
    }

    droneLastActivityMs = now;
  }
  lastFlowlapseFrameModeToggleComboActive = frameModeToggleComboActive;
  if (frameModeToggleComboActive) {
    stopAllMotors();
    return true;
  }

  bool stickSpeedToggleComboActive = ps2x.Button(PSB_START)
      && ps2x.Button(PSB_SELECT)
      && ps2x.Button(PSB_CROSS);
  if (stickSpeedToggleComboActive && !lastDroneStickSpeedToggleComboActive) {
    bool toggleAllowed = (flowlapseState != FLOWLAPSE_STATE_CAPTURE_RUNNING)
                      && (flowlapseState != FLOWLAPSE_STATE_CAPTURE_PAUSED);
    suppressDroneNextSelectRelease = true;
    suppressDroneNextStartRelease = true;

    if (!toggleAllowed) {
      startLockoutDeniedRumbleFeedback();
      Serial.println(F("Drone: stick speed mode toggle blocked while capture is running/paused."));
    } else {
      droneProportionalStickSpeedEnabled = !droneProportionalStickSpeedEnabled;
      startFeedbackRumble(
          droneProportionalStickSpeedEnabled
              ? DRONE_STICK_SPEED_TOGGLE_PROPORTIONAL_PULSES
              : DRONE_STICK_SPEED_TOGGLE_FIXED_PULSES,
          DRONE_STICK_SPEED_TOGGLE_RUMBLE_ON_MS,
          DRONE_STICK_SPEED_TOGGLE_RUMBLE_TOTAL_MS);
      broadcastStatus(droneProportionalStickSpeedEnabled
          ? "DRONE_STICK_SPEED:PROPORTIONAL"
          : "DRONE_STICK_SPEED:FIXED");
      Serial.print(F("Drone: stick speed mode "));
      Serial.println(droneProportionalStickSpeedEnabled ? F("PROPORTIONAL") : F("FIXED"));
    }

    droneLastActivityMs = now;
  }
  lastDroneStickSpeedToggleComboActive = stickSpeedToggleComboActive;
  if (stickSpeedToggleComboActive) {
    stopAllMotors();
    return true;
  }

  bool accelProfileCycleEligible = (flowlapseState == FLOWLAPSE_STATE_READY_FOR_PREVIEW
                                 || flowlapseState == FLOWLAPSE_STATE_READY_FOR_CAPTURE);
  bool noAccelProfileConflictButtons = !ps2x.Button(PSB_START)
                                    && !ps2x.Button(PSB_CROSS)
                                    && !ps2x.Button(PSB_PAD_UP)
                                    && !ps2x.Button(PSB_PAD_DOWN)
                                    && !ps2x.Button(PSB_PAD_LEFT)
                                    && !ps2x.Button(PSB_PAD_RIGHT);
  bool accelProfileCycleChordActive = accelProfileCycleEligible && ps2x.Button(PSB_SELECT) && ps2x.Button(PSB_TRIANGLE) && noAccelProfileConflictButtons;
  if (accelProfileCycleChordActive) {
    stopAllMotors();
  }
  bool currentTriangleButtonState = ps2x.Button(PSB_TRIANGLE);
  bool triangleJustReleased = lastTriangleButtonState && !currentTriangleButtonState;
  lastTriangleButtonState = currentTriangleButtonState;
  
  if (accelProfileCycleEligible && triangleJustReleased && ps2x.Button(PSB_SELECT) && noAccelProfileConflictButtons) {
    flowlapseAccelerationProfile = static_cast<FlowlapseAccelerationProfile>(
        (static_cast<uint8_t>(flowlapseAccelerationProfile) + 1) % static_cast<uint8_t>(FLOWLAPSE_ACCEL_PROFILE_COUNT));
    suppressDroneNextSelectRelease = true;
    startFeedbackRumble(static_cast<uint8_t>(flowlapseAccelerationProfile) + 1,
                        FLOWLAPSE_WAYPOINT_RUMBLE_ON_MS,
                        FLOWLAPSE_WAYPOINT_RUMBLE_TOTAL_MS);
    Serial.print(F("Flowlapse: acceleration profile = "));
    Serial.println(getFlowlapseAccelerationProfileName(flowlapseAccelerationProfile));
    droneLastActivityMs = now;
    return true;
  }
  if (accelProfileCycleChordActive) {
    return true;
  }

  bool flowlapseClearComboActive = ps2x.Button(PSB_L1) && ps2x.Button(PSB_R1);
  if (flowlapseClearComboActive && !lastFlowlapseClearComboActive) {
    if (flowlapseState == FLOWLAPSE_STATE_PREVIEW_RUNNING
        || flowlapseState == FLOWLAPSE_STATE_CAPTURE_RUNNING
        || flowlapseState == FLOWLAPSE_STATE_CAPTURE_PAUSED
        || flowlapseState == FLOWLAPSE_STATE_UNDO_RUNNING
        || flowlapseState == FLOWLAPSE_STATE_JOG_RUNNING) {
      startLockoutDeniedRumbleFeedback();
      Serial.println(F("Flowlapse: cannot wipe while preview/capture/undo/jog is running."));
    } else {
      resetFlowlapseSession(false);
      stopAllMotors();
      startFeedbackRumble(FLOWLAPSE_WIPE_RUMBLE_PULSES,
                          FLOWLAPSE_WIPE_RUMBLE_ON_MS,
                          FLOWLAPSE_WIPE_RUMBLE_TOTAL_MS);
      Serial.println(F("Flowlapse: full course wiped. Recording re-armed."));
      broadcastStatus("Flowlapse: full course wiped. Recording re-armed.");
    }

    droneLastActivityMs = now;
  }
  lastFlowlapseClearComboActive = flowlapseClearComboActive;

  bool flowlapseDeleteLastComboActive = ps2x.Button(PSB_L2) && ps2x.Button(PSB_R2);
  if (flowlapseDeleteLastComboActive && !lastFlowlapseDeleteLastComboActive) {
    if (flowlapseState == FLOWLAPSE_STATE_PREVIEW_RUNNING
        || flowlapseState == FLOWLAPSE_STATE_CAPTURE_RUNNING
        || flowlapseState == FLOWLAPSE_STATE_CAPTURE_PAUSED
        || flowlapseState == FLOWLAPSE_STATE_UNDO_RUNNING) {
      startLockoutDeniedRumbleFeedback();
      Serial.println(F("Flowlapse: cannot undo while preview/capture/undo is running."));
    } else if (flowlapseWaypointCount == 0) {
      startLockoutDeniedRumbleFeedback();
      Serial.println(F("Flowlapse: no waypoint to undo."));
    } else if (flowlapseState == FLOWLAPSE_STATE_UNDO_READY_DELETE) {
      flowlapseWaypointCount--;
      invalidateFlowlapseArcLengthLut();
      startFeedbackRumble(1, FLOWLAPSE_WAYPOINT_RUMBLE_ON_MS, FLOWLAPSE_WAYPOINT_RUMBLE_TOTAL_MS);

      broadcastFlowlapseWaypointCount();

      if (flowlapseWaypointCount < 2) {
        flowlapseState = FLOWLAPSE_STATE_RECORDING;
        Serial.print(F("Flowlapse: last waypoint deleted. Remaining "));
        Serial.print(flowlapseWaypointCount);
        Serial.println(F(" waypoint(s); keep recording."));
      } else {
        flowlapseState = FLOWLAPSE_STATE_READY_FOR_PREVIEW;
        Serial.print(F("Flowlapse: last waypoint deleted. Remaining "));
        Serial.print(flowlapseWaypointCount);
        Serial.println(F(" waypoints; run SELECT preview again."));
      }
    } else {
      startFlowlapseUndoLastWaypoint(now);
    }

    droneLastActivityMs = now;
  }
  lastFlowlapseDeleteLastComboActive = flowlapseDeleteLastComboActive;

  bool dwellAdjUpComboActive   = ps2x.Button(PSB_SELECT) && ps2x.Button(PSB_PAD_UP);
  bool dwellAdjDownComboActive = ps2x.Button(PSB_SELECT) && ps2x.Button(PSB_PAD_DOWN);
  if (dwellAdjUpComboActive && !lastDwellAdjustUpComboActive) {
    adjustFlowlapseDwell(static_cast<long>(FLOWLAPSE_DWELL_ADJUST_INCREMENT_MS));
    suppressDroneNextSelectRelease = true;
    droneLastActivityMs = now;
  }
  if (dwellAdjDownComboActive && !lastDwellAdjustDownComboActive) {
    adjustFlowlapseDwell(-static_cast<long>(FLOWLAPSE_DWELL_ADJUST_INCREMENT_MS));
    suppressDroneNextSelectRelease = true;
    droneLastActivityMs = now;
  }
  lastDwellAdjustUpComboActive   = dwellAdjUpComboActive;
  lastDwellAdjustDownComboActive = dwellAdjDownComboActive;

  bool homeSetComboActive = ps2x.Button(PSB_START) && ps2x.Button(PSB_TRIANGLE);
  if (homeSetComboActive && !lastHomeSetComboActive) {
    if (isFlowlapseBusyForHomeAction()) {
      startLockoutDeniedRumbleFeedback();
      Serial.println(F("Home set denied: flowlapse task is active."));
      broadcastStatus("HOME:BLOCKED");
    } else {
      setHomePoseFromCurrentPose(true);
      suppressDroneNextStartRelease = true;
      startFeedbackRumble(1, FLOWLAPSE_WAYPOINT_RUMBLE_ON_MS, FLOWLAPSE_WAYPOINT_RUMBLE_TOTAL_MS);
      droneLastActivityMs = now;
    }
  }
  lastHomeSetComboActive = homeSetComboActive;

  bool homeGoComboActive = ps2x.Button(PSB_START) && ps2x.Button(PSB_SQUARE);
  if (homeGoComboActive && !lastHomeGoComboActive) {
    suppressDroneNextStartRelease = true;
    if (startHomeReturn(now)) {
      droneLastActivityMs = now;
    }
  }
  lastHomeGoComboActive = homeGoComboActive;

  bool homeClearComboActive = ps2x.Button(PSB_START) && ps2x.Button(PSB_CIRCLE);
  if (homeClearComboActive && !lastHomeClearComboActive) {
    if (isFlowlapseBusyForHomeAction()) {
      startLockoutDeniedRumbleFeedback();
      Serial.println(F("Home clear denied: flowlapse task is active."));
      broadcastStatus("HOME:BLOCKED");
    } else {
      clearHomePose(true);
      suppressDroneNextStartRelease = true;
      startFeedbackRumble(2, FLOWLAPSE_WAYPOINT_RUMBLE_ON_MS, FLOWLAPSE_WAYPOINT_RUMBLE_TOTAL_MS);
      droneLastActivityMs = now;
    }
  }
  lastHomeClearComboActive = homeClearComboActive;

  bool l3HoldEligible = (flowlapseState != FLOWLAPSE_STATE_PREVIEW_RUNNING
                      && flowlapseState != FLOWLAPSE_STATE_CAPTURE_RUNNING
                      && flowlapseState != FLOWLAPSE_STATE_UNDO_RUNNING
                      && flowlapseState != FLOWLAPSE_STATE_JOG_RUNNING);
  if (l3HoldEligible && ps2x.Button(PSB_L3)) {
    if (!flowlapseL3HoldActive) {
      flowlapseL3HoldActive = true;
      flowlapseL3HoldStartMs = now;
    } else if (now - flowlapseL3HoldStartMs >= FLOWLAPSE_L3_RESET_HOLD_MS) {
      resetFlowlapseSession(false);
      stopAllMotors();
      startFeedbackRumble(2, FLOWLAPSE_WAYPOINT_RUMBLE_ON_MS, FLOWLAPSE_WAYPOINT_RUMBLE_TOTAL_MS);
      Serial.println(F("Flowlapse: L3 hold reset — course cleared, recording re-armed."));
      broadcastStatus("Flowlapse: L3 hold reset - course cleared, recording re-armed.");
      flowlapseL3HoldActive = false;
      flowlapseL3HoldStartMs = 0;
      suppressDroneNextL3Release = true;
      droneLastActivityMs = now;
    }
  } else {
    flowlapseL3HoldActive = false;
    flowlapseL3HoldStartMs = 0;
  }

  bool currentL3ButtonState = ps2x.Button(PSB_L3);
  bool l3JustReleased = lastL3ButtonState && !currentL3ButtonState;
  lastL3ButtonState = currentL3ButtonState;
  
  if (l3JustReleased) {
    if (suppressDroneNextL3Release) {
      suppressDroneNextL3Release = false;
    } else if (flowlapseState == FLOWLAPSE_STATE_RECORDING) {
      captureFlowlapseWaypoint();
      droneLastActivityMs = now;
    }
  }

  bool jogReadyState = (flowlapseState == FLOWLAPSE_STATE_READY_FOR_PREVIEW
                     || flowlapseState == FLOWLAPSE_STATE_READY_FOR_CAPTURE);
  bool waypointDwellAdjUpComboActive = jogReadyState && ps2x.Button(PSB_TRIANGLE) && ps2x.Button(PSB_PAD_UP);
  bool waypointDwellAdjDownComboActive = jogReadyState && ps2x.Button(PSB_TRIANGLE) && ps2x.Button(PSB_PAD_DOWN);

  if (waypointDwellAdjUpComboActive && !lastWaypointDwellAdjustUpComboActive) {
    adjustFlowlapseWaypointDwell(flowlapseJogIndex, static_cast<long>(FLOWLAPSE_DWELL_ADJUST_INCREMENT_MS));
    droneLastActivityMs = now;
  }
  if (waypointDwellAdjDownComboActive && !lastWaypointDwellAdjustDownComboActive) {
    adjustFlowlapseWaypointDwell(flowlapseJogIndex, -static_cast<long>(FLOWLAPSE_DWELL_ADJUST_INCREMENT_MS));
    droneLastActivityMs = now;
  }
  lastWaypointDwellAdjustUpComboActive = waypointDwellAdjUpComboActive;
  lastWaypointDwellAdjustDownComboActive = waypointDwellAdjDownComboActive;
  
  bool currentPadRightButtonState = ps2x.Button(PSB_PAD_RIGHT);
  bool padRightJustReleased = lastPadRightButtonState && !currentPadRightButtonState;
  lastPadRightButtonState = currentPadRightButtonState;
  
  if (jogReadyState && padRightJustReleased) {
    if (flowlapseJogIndex < static_cast<uint8_t>(flowlapseWaypointCount - 1)) {
      flowlapseJogIndex++;
      flowlapseState = FLOWLAPSE_STATE_JOG_RUNNING;
      resetFlowlapseAxisTierState(now);
      Serial.print(F("Flowlapse: jog -> waypoint "));
      Serial.println(flowlapseJogIndex + 1);
      Serial.print(F("Flowlapse: jog target "));
      Serial.print(flowlapseJogIndex + 1);
      Serial.print(F("/"));
      Serial.println(flowlapseWaypointCount);
      Serial.print(F("Flowlapse: selected waypoint dwell (ms) = "));
      Serial.println(flowlapseWaypoints[flowlapseJogIndex].dwellMs);
    } else {
      startLockoutDeniedRumbleFeedback();
      Serial.println(F("Flowlapse: already at last waypoint."));
    }
    droneLastActivityMs = now;
  }
  bool currentPadLeftButtonState = ps2x.Button(PSB_PAD_LEFT);
  bool padLeftJustReleased = lastPadLeftButtonState && !currentPadLeftButtonState;
  lastPadLeftButtonState = currentPadLeftButtonState;
  
  if (jogReadyState && padLeftJustReleased) {
    if (flowlapseJogIndex > 0) {
      flowlapseJogIndex--;
      flowlapseState = FLOWLAPSE_STATE_JOG_RUNNING;
      resetFlowlapseAxisTierState(now);
      Serial.print(F("Flowlapse: jog -> waypoint "));
      Serial.println(flowlapseJogIndex + 1);
      Serial.print(F("Flowlapse: jog target "));
      Serial.print(flowlapseJogIndex + 1);
      Serial.print(F("/"));
      Serial.println(flowlapseWaypointCount);
      Serial.print(F("Flowlapse: selected waypoint dwell (ms) = "));
      Serial.println(flowlapseWaypoints[flowlapseJogIndex].dwellMs);
    } else {
      startLockoutDeniedRumbleFeedback();
      Serial.println(F("Flowlapse: already at first waypoint."));
    }
    droneLastActivityMs = now;
  }

  bool flowlapseReturnToStartComboActive = ps2x.Button(PSB_START) && ps2x.Button(PSB_R1);
  if (flowlapseReturnToStartComboActive && !lastFlowlapseReturnToStartComboActive) {
    if (flowlapseState == FLOWLAPSE_STATE_PREVIEW_RUNNING
        || flowlapseState == FLOWLAPSE_STATE_CAPTURE_RUNNING
        || flowlapseState == FLOWLAPSE_STATE_CAPTURE_PAUSED
        || flowlapseState == FLOWLAPSE_STATE_UNDO_RUNNING
        || flowlapseState == FLOWLAPSE_STATE_JOG_RUNNING) {
      startLockoutDeniedRumbleFeedback();
      Serial.println(F("Flowlapse: cannot return-to-start while preview/capture/undo/jog is running."));
    } else if (flowlapseWaypointCount == 0) {
      startLockoutDeniedRumbleFeedback();
      Serial.println(F("Flowlapse: no waypoints recorded yet."));
    } else if (flowlapseJogIndex == 0) {
      startLockoutDeniedRumbleFeedback();
      Serial.println(F("Flowlapse: already at first waypoint."));
    } else {
      flowlapseJogIndex = 0;
      flowlapseState = FLOWLAPSE_STATE_JOG_RUNNING;
      resetFlowlapseAxisTierState(now);
      suppressDroneNextStartRelease = true;
      startFeedbackRumble(1, FLOWLAPSE_WAYPOINT_RUMBLE_ON_MS, FLOWLAPSE_WAYPOINT_RUMBLE_TOTAL_MS);
      Serial.println(F("Flowlapse: returning to first waypoint via START+R1."));
    }
    droneLastActivityMs = now;
  }
  lastFlowlapseReturnToStartComboActive = flowlapseReturnToStartComboActive;

  bool currentDroneSelectButtonState = ps2x.Button(PSB_SELECT);
  bool droneSelectJustReleased = lastDroneSelectButtonState && !currentDroneSelectButtonState;
  lastDroneSelectButtonState = currentDroneSelectButtonState;
  
  if (droneSelectJustReleased) {
    if (suppressDroneNextSelectRelease) {
      suppressDroneNextSelectRelease = false;
    } else {
    if (flowlapseState == FLOWLAPSE_STATE_RECORDING) {
      if (flowlapseWaypointCount < 2) {
        startLockoutDeniedRumbleFeedback();
        Serial.println(F("Flowlapse: record at least 2 waypoints before stopping record."));
      } else {
        flowlapseState = FLOWLAPSE_STATE_READY_FOR_PREVIEW;
        stopAllMotors();
        startFeedbackRumble(flowlapseWaypointCount, FLOWLAPSE_WAYPOINT_RUMBLE_ON_MS, FLOWLAPSE_WAYPOINT_RUMBLE_TOTAL_MS);
        Serial.println(F("Flowlapse: recording stopped. Press SELECT again for preview."));
        broadcastStatus("Flowlapse: recording stopped. Press SELECT again for preview.");
      }
      droneLastActivityMs = now;
    } else if (flowlapseState == FLOWLAPSE_STATE_READY_FOR_PREVIEW || flowlapseState == FLOWLAPSE_STATE_READY_FOR_CAPTURE) {
      startFlowlapsePreview();
      droneLastActivityMs = now;
    } else if (flowlapseState == FLOWLAPSE_STATE_CAPTURE_PAUSED) {
      // Abort: pause with START first, then SELECT to confirm cancel
      stopAllMotors();
      flowlapseState = FLOWLAPSE_STATE_READY_FOR_CAPTURE;
      flowlapseCaptureDirection = 1;
      flowlapseTargetWaypointIndex = 0;
      startFeedbackRumble(3, FLOWLAPSE_WAYPOINT_RUMBLE_ON_MS, FLOWLAPSE_WAYPOINT_RUMBLE_TOTAL_MS);
      Serial.println(F("Flowlapse: capture aborted. Waypoints preserved — press SELECT to re-run preview."));
      broadcastStatus("Flowlapse: capture aborted. Waypoints preserved.");
      broadcastStatus("CAPTURE_STATE:READY");
      droneLastActivityMs = now;
    }
  }
  }

  bool currentDroneStartButtonState = ps2x.Button(PSB_START);
  bool droneStartJustReleased = lastDroneStartButtonState && !currentDroneStartButtonState;
  lastDroneStartButtonState = currentDroneStartButtonState;
  
  if (droneStartJustReleased) {
    if (suppressDroneNextStartRelease) {
      suppressDroneNextStartRelease = false;
    } else {
      if (flowlapseState == FLOWLAPSE_STATE_PREVIEW_RUNNING) {
        completeFlowlapsePreview();
        Serial.println(F("Flowlapse: preview skipped by START; capture is now ready."));
        droneLastActivityMs = now;
      } else if (flowlapseState == FLOWLAPSE_STATE_CAPTURE_RUNNING) {
        // Pause capture
        stopAllMotors();
        flowlapsePauseStartMs = now;
        flowlapseState = FLOWLAPSE_STATE_CAPTURE_PAUSED;
        startFeedbackRumble(1, FLOWLAPSE_WAYPOINT_RUMBLE_ON_MS, FLOWLAPSE_WAYPOINT_RUMBLE_TOTAL_MS);
        const char* pauseMsg = "Flowlapse: capture paused. Press START again to resume.";
        Serial.println(pauseMsg);
        broadcastStatus(pauseMsg);
        broadcastStatus("CAPTURE_STATE:PAUSED");
        droneLastActivityMs = now;
      } else if (flowlapseState == FLOWLAPSE_STATE_CAPTURE_PAUSED) {
        // Resume capture — compensate timestamps for time spent paused
        if (flowlapsePauseStartMs > 0 && now >= flowlapsePauseStartMs) {
          unsigned long pausedDurationMs = now - flowlapsePauseStartMs;
          flowlapseTotalPausedMs += pausedDurationMs;
          flowlapseCapturePhaseStartMs += pausedDurationMs;
          flowlapsePauseStartMs = 0;
        }
        flowlapseState = FLOWLAPSE_STATE_CAPTURE_RUNNING;
        startFeedbackRumble(1, FLOWLAPSE_WAYPOINT_RUMBLE_ON_MS, FLOWLAPSE_WAYPOINT_RUMBLE_TOTAL_MS);
        const char* resumeMsg = "Flowlapse: capture resumed.";
        Serial.println(resumeMsg);
        broadcastStatus(resumeMsg);
        broadcastStatus("CAPTURE_STATE:RUNNING");
        droneLastActivityMs = now;
      } else if (flowlapseState == FLOWLAPSE_STATE_READY_FOR_CAPTURE) {
        startFlowlapseCapture(now);
        droneLastActivityMs = now;
      } else if (flowlapseState == FLOWLAPSE_STATE_READY_FOR_PREVIEW) {
        startLockoutDeniedRumbleFeedback();
        Serial.println(F("Flowlapse: run preview first (SELECT), then press START for capture."));
        droneLastActivityMs = now;
      }
    }
  }

  return false;
}

void handleDroneFlowlapseWorkflow(unsigned long now, float deltaSeconds) {
  if (homeReturnActive) {
    handleHomeReturnStep(now, deltaSeconds);
    droneLastActivityMs = now;
    return;
  }

  if (handleDroneFlowlapseButtons(now)) {
    return;
  }

  if (flowlapseState == FLOWLAPSE_STATE_PREVIEW_RUNNING) {
    handleFlowlapsePreviewStep(now, deltaSeconds);
    droneLastActivityMs = now;
    return;
  }

  if (flowlapseState == FLOWLAPSE_STATE_CAPTURE_RUNNING) {
    handleFlowlapseCaptureStep(now, deltaSeconds);
    logDroneSpeedModifierStateIfChanged();
    droneLastActivityMs = now;
    return;
  }

  if (flowlapseState == FLOWLAPSE_STATE_CAPTURE_PAUSED) {
    stopAllMotors();
    logDroneSpeedModifierStateIfChanged();
    droneLastActivityMs = now;
    return;
  }

  if (flowlapseState == FLOWLAPSE_STATE_UNDO_RUNNING) {
    handleFlowlapseUndoStep(now, deltaSeconds);
    droneLastActivityMs = now;
    return;
  }

  if (flowlapseState == FLOWLAPSE_STATE_UNDO_READY_DELETE) {
    stopAllMotors();
    droneLastActivityMs = now;
    return;
  }

  if (flowlapseState == FLOWLAPSE_STATE_JOG_RUNNING) {
    if (flowlapseJogIndex < flowlapseWaypointCount) {
      const FlowlapseWaypoint& jogTarget = flowlapseWaypoints[flowlapseJogIndex];
      applyFlowlapseMotionTowardWaypoint(jogTarget, now, deltaSeconds);
      if (isFlowlapseTargetReached(jogTarget)) {
        stopAllMotors();
        flowlapseState = FLOWLAPSE_STATE_RECORDING;
        startFeedbackRumble(1, FLOWLAPSE_WAYPOINT_RUMBLE_ON_MS, FLOWLAPSE_WAYPOINT_RUMBLE_TOTAL_MS);
        Serial.print(F("Flowlapse: jog reached waypoint "));
        Serial.print(flowlapseJogIndex + 1);
        Serial.print(F("/"));
        Serial.println(flowlapseWaypointCount);
        broadcastStatus("MODE:DRONE");
      }
    } else {
      stopAllMotors();
      flowlapseState = FLOWLAPSE_STATE_RECORDING;
      broadcastStatus("MODE:DRONE");
    }
    droneLastActivityMs = now;
    return;
  }

  if (flowlapseState == FLOWLAPSE_STATE_RECORDING) {
    handleDroneStickControl();
    updateFlowlapseEstimatedPositionFromManualSticks(deltaSeconds);
    handleFocusAxis();
    return;
  }

  if (flowlapseState == FLOWLAPSE_STATE_READY_FOR_PREVIEW
      || flowlapseState == FLOWLAPSE_STATE_READY_FOR_CAPTURE) {
    stopAllMotors();
    logDroneSpeedModifierStateIfChanged();
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
  frameCountCompleteRumblePhase = FRAMECOUNT_COMPLETE_RUMBLE_IDLE;
  frameCountCompleteRumblePhaseStartMs = 0;
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

void startFrameCountCompleteRumbleFeedback() {
  stopIntervalRumbleFeedback();
  frameCountCompleteRumblePhase = FRAMECOUNT_COMPLETE_RUMBLE_LONG_ON;
  frameCountCompleteRumblePhaseStartMs = millis();
  vibrate = RUMBLE_ACTIVE_INTENSITY;
}

void startSettingsReplayRumble() {
  chainStepDistAfterInterval = true;
  startIntervalRumbleFeedback();
  Serial.print(F("Settings replay: interval="));
  Serial.print(timelapseIntervalSeconds);
  Serial.print(F("s, stepDist="));
  Serial.print(stepDist);
  Serial.println(F("ms"));
}

void startIntervalRumbleFeedbackNow() {
  stepDistRumblePhase = STEP_DIST_RUMBLE_IDLE;
  stepDistRumblePhaseStartMs = 0;
  stepDistRumbleLongsRemaining = 0;

  intervalRumbleLongsRemaining = timelapseIntervalSeconds / 10;
  intervalRumbleShortsRemaining = timelapseIntervalSeconds % 10;

  // Manual compensation requested: add one pulse to active interval group.
  if (intervalRumbleShortsRemaining > 0) {
    intervalRumbleShortsRemaining++;
  } else if (intervalRumbleLongsRemaining > 0) {
    intervalRumbleLongsRemaining++;
  }

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
  if (frameCountCompleteRumblePhase != FRAMECOUNT_COMPLETE_RUMBLE_IDLE) {
    switch (frameCountCompleteRumblePhase) {
      case FRAMECOUNT_COMPLETE_RUMBLE_LONG_ON:
        vibrate = RUMBLE_ACTIVE_INTENSITY;
        if (now - frameCountCompleteRumblePhaseStartMs >= FRAMECOUNT_COMPLETE_RUMBLE_LONG_ON_MS) {
          frameCountCompleteRumblePhase = FRAMECOUNT_COMPLETE_RUMBLE_LONG_PAUSE;
          frameCountCompleteRumblePhaseStartMs = now;
          vibrate = 0;
        }
        break;
      case FRAMECOUNT_COMPLETE_RUMBLE_LONG_PAUSE:
        vibrate = 0;
        if (now - frameCountCompleteRumblePhaseStartMs >= (FRAMECOUNT_COMPLETE_RUMBLE_LONG_TOTAL_MS - FRAMECOUNT_COMPLETE_RUMBLE_LONG_ON_MS)) {
          frameCountCompleteRumblePhase = FRAMECOUNT_COMPLETE_RUMBLE_SHORT_ON;
          frameCountCompleteRumblePhaseStartMs = now;
        }
        break;
      case FRAMECOUNT_COMPLETE_RUMBLE_SHORT_ON:
        vibrate = RUMBLE_ACTIVE_INTENSITY;
        if (now - frameCountCompleteRumblePhaseStartMs >= FRAMECOUNT_COMPLETE_RUMBLE_SHORT_ON_MS) {
          frameCountCompleteRumblePhase = FRAMECOUNT_COMPLETE_RUMBLE_SHORT_PAUSE;
          frameCountCompleteRumblePhaseStartMs = now;
          vibrate = 0;
        }
        break;
      case FRAMECOUNT_COMPLETE_RUMBLE_SHORT_PAUSE:
        vibrate = 0;
        if (now - frameCountCompleteRumblePhaseStartMs >= (FRAMECOUNT_COMPLETE_RUMBLE_SHORT_TOTAL_MS - FRAMECOUNT_COMPLETE_RUMBLE_SHORT_ON_MS)) {
          stopIntervalRumbleFeedback();
        }
        break;
      case FRAMECOUNT_COMPLETE_RUMBLE_IDLE:
        break;
    }
    return;
  }

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
          intervalRumblePhase = INTERVAL_RUMBLE_LONG_PAUSE;
          intervalRumblePhaseStartMs = now;
          vibrate = 0;
        } else if (intervalRumbleShortsRemaining > 0) {
          intervalRumblePhase = INTERVAL_RUMBLE_LONG_PAUSE;
          intervalRumblePhaseStartMs = now;
          vibrate = 0;
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
    case INTERVAL_RUMBLE_LONG_PAUSE:
      vibrate = 0;
      if (intervalRumbleLongsRemaining > 0) {
        if (now - intervalRumblePhaseStartMs >= INTERVAL_RUMBLE_LONG_PAUSE_MS) {
          intervalRumblePhase = INTERVAL_RUMBLE_LONG_ACTIVE;
          intervalRumblePhaseStartMs = now;
        }
      } else if (intervalRumbleShortsRemaining > 0) {
        if (now - intervalRumblePhaseStartMs >= INTERVAL_RUMBLE_GROUP_SEPARATOR_MS) {
          intervalRumblePhase = INTERVAL_RUMBLE_SHORT_ACTIVE;
          intervalRumblePhaseStartMs = now;
        }
      } else {
        if (chainStepDistAfterInterval) {
          chainStepDistAfterInterval = false;
          startRumbleSeparator(PENDING_RUMBLE_STEP_DIST);
        } else {
          stopIntervalRumbleFeedback();
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
    Serial.println(F("Timelapse interval limit reached."));
    return;
  }

  timelapseIntervalSeconds = newIntervalSeconds;
  updateIntervalMs();
  enforceTimelapseTimingBudget(true);
  savePersistedSettings();
  Serial.print(F("Timelapse interval (seconds) = "));
  Serial.println(timelapseIntervalSeconds);
  char intervalLine[40];
  snprintf(intervalLine, sizeof(intervalLine), "TIMELAPSE_INTERVAL:%u",
           static_cast<unsigned int>(timelapseIntervalSeconds));
  broadcastStatus(intervalLine);
  startIntervalRumbleFeedback();
}

unsigned long getTimelapseEffectiveMoveDurationMs() {
  unsigned long moveDurationMs = static_cast<unsigned long>(stepDist);
  unsigned long antiBacklashMs = timelapseAntiBacklashEnabled
      ? static_cast<unsigned long>(TIMELAPSE_ANTI_BACKLASH_DURATION_MS)
      : 0UL;
  unsigned long rampEnvelopeMs = static_cast<unsigned long>(TIMELAPSE_RAMP_UP_DURATION_MS)
      + static_cast<unsigned long>(TIMELAPSE_RAMP_DOWN_DURATION_MS)
      + static_cast<unsigned long>(TIMELAPSE_CRUISE_MIN_DURATION_MS);
  rampEnvelopeMs += antiBacklashMs;
  if (moveDurationMs < rampEnvelopeMs) {
    moveDurationMs = rampEnvelopeMs;
  }
  return moveDurationMs;
}

void enforceTimelapseTimingBudget(bool announceClamps) {
  unsigned long effectiveMoveMs = getTimelapseEffectiveMoveDurationMs();
  unsigned long maxDwellByGuard = (timelapseIntervalMs > effectiveMoveMs)
      ? (timelapseIntervalMs - effectiveMoveMs)
      : 0UL;

  unsigned long clampedMaxDwell = constrain(maxDwellByGuard,
      TIMELAPSE_SETTLE_DWELL_MIN_MS,
      TIMELAPSE_SETTLE_DWELL_MAX_MS);

  if (timelapseSettleDwellMs > clampedMaxDwell) {
    timelapseSettleDwellMs = clampedMaxDwell;
    if (announceClamps) {
      startLimitReachedRumbleFeedback();
      Serial.print(F("Timelapse dwell clamped by timing budget to "));
      Serial.print(timelapseSettleDwellMs);
      Serial.println(F(" ms."));
      char dwellClampLine[48];
      snprintf(dwellClampLine, sizeof(dwellClampLine), "TIMELAPSE_DWELL:%lu", timelapseSettleDwellMs);
      broadcastStatus(dwellClampLine);
    }
  }
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
    Serial.println(F("Timelapse stepDist limit reached."));
    return;
  }

  stepDist = newStepDist;
  enforceTimelapseTimingBudget(true);
  savePersistedSettings();
  Serial.print(F("Timelapse stepDist (ms) = "));
  Serial.println(stepDist);
  char stepDistLine[40];
  snprintf(stepDistLine, sizeof(stepDistLine), "TIMELAPSE_STEPDIST:%d", stepDist);
  broadcastStatus(stepDistLine);
  startStepDistRumbleFeedback();
}

void adjustTimelapseSettleDwell(int deltaMs) {
  long newDwellMs = static_cast<long>(timelapseSettleDwellMs) + deltaMs;
  if (newDwellMs < static_cast<long>(TIMELAPSE_SETTLE_DWELL_MIN_MS)) {
    newDwellMs = static_cast<long>(TIMELAPSE_SETTLE_DWELL_MIN_MS);
  }
  if (newDwellMs > static_cast<long>(TIMELAPSE_SETTLE_DWELL_MAX_MS)) {
    newDwellMs = static_cast<long>(TIMELAPSE_SETTLE_DWELL_MAX_MS);
  }
  if (static_cast<unsigned long>(newDwellMs) == timelapseSettleDwellMs) {
    startLimitReachedRumbleFeedback();
    Serial.println(F("Timelapse dwell limit reached."));
    return;
  }

  timelapseSettleDwellMs = static_cast<unsigned long>(newDwellMs);
  enforceTimelapseTimingBudget(true);
  savePersistedSettings();

  Serial.print(F("Timelapse settle dwell (ms) = "));
  Serial.println(timelapseSettleDwellMs);
  char dwellLine[48];
  snprintf(dwellLine, sizeof(dwellLine), "TIMELAPSE_DWELL:%lu", timelapseSettleDwellMs);
  broadcastStatus(dwellLine);
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

uint8_t computePersistedSettingsChecksum(const PersistedSettingsV1& settings) {
  return static_cast<uint8_t>((settings.magic & 0xFF)
      + (settings.magic >> 8)
      + settings.version
      + settings.timelapseIntervalSeconds);
}

uint8_t computePersistedSettingsChecksum(const PersistedSettingsV2& settings) {
  return static_cast<uint8_t>((settings.magic & 0xFF)
      + (settings.magic >> 8)
      + settings.version
      + settings.timelapseIntervalSeconds
      + (settings.stepDist & 0xFF)
      + ((settings.stepDist >> 8) & 0xFF));
}

uint8_t computePersistedSettingsChecksum(const PersistedSettingsV3& settings) {
  return static_cast<uint8_t>((settings.magic & 0xFF)
      + (settings.magic >> 8)
      + settings.version
      + settings.timelapseIntervalSeconds
      + (settings.stepDist & 0xFF)
      + ((settings.stepDist >> 8) & 0xFF)
      + settings.flags);
}

uint8_t computePersistedSettingsChecksum(const PersistedSettingsV4& settings) {
  uint8_t sum = static_cast<uint8_t>((settings.magic & 0xFF)
      + (settings.magic >> 8)
      + settings.version
      + settings.timelapseIntervalSeconds
      + (settings.stepDist & 0xFF)
      + ((settings.stepDist >> 8) & 0xFF)
      + settings.flags);

  const uint8_t* homeSwingBytes = reinterpret_cast<const uint8_t*>(&settings.homeSwing);
  const uint8_t* homeLiftBytes = reinterpret_cast<const uint8_t*>(&settings.homeLift);
  const uint8_t* homePanBytes = reinterpret_cast<const uint8_t*>(&settings.homePan);
  const uint8_t* homeTiltBytes = reinterpret_cast<const uint8_t*>(&settings.homeTilt);
  for (uint8_t i = 0; i < sizeof(float); ++i) {
    sum = static_cast<uint8_t>(sum + homeSwingBytes[i] + homeLiftBytes[i] + homePanBytes[i] + homeTiltBytes[i]);
  }
  return sum;
}

uint8_t computePersistedSettingsChecksum(const PersistedSettingsV5& settings) {
  uint8_t sum = static_cast<uint8_t>((settings.magic & 0xFF)
      + (settings.magic >> 8)
      + settings.version
      + settings.timelapseIntervalSeconds
      + (settings.stepDist & 0xFF)
      + ((settings.stepDist >> 8) & 0xFF)
      + (settings.timelapseSettleDwellMs & 0xFF)
      + ((settings.timelapseSettleDwellMs >> 8) & 0xFF)
      + settings.flags);

  const uint8_t* homeSwingBytes = reinterpret_cast<const uint8_t*>(&settings.homeSwing);
  const uint8_t* homeLiftBytes = reinterpret_cast<const uint8_t*>(&settings.homeLift);
  const uint8_t* homePanBytes = reinterpret_cast<const uint8_t*>(&settings.homePan);
  const uint8_t* homeTiltBytes = reinterpret_cast<const uint8_t*>(&settings.homeTilt);
  for (uint8_t i = 0; i < sizeof(float); ++i) {
    sum = static_cast<uint8_t>(sum + homeSwingBytes[i] + homeLiftBytes[i] + homePanBytes[i] + homeTiltBytes[i]);
  }
  return sum;
}

void writePersistedSettings(const PersistedSettingsV5& settings) {
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&settings);
  for (unsigned int i = 0; i < sizeof(PersistedSettingsV5); ++i) {
    EEPROM.update(EEPROM_SETTINGS_ADDRESS + static_cast<int>(i), bytes[i]);
  }
}

bool loadPersistedSettings() {
  PersistedSettingsV5 settingsV5;
  EEPROM.get(EEPROM_SETTINGS_ADDRESS, settingsV5);

  if (settingsV5.magic == PERSISTED_SETTINGS_MAGIC
      && settingsV5.version == PERSISTED_SETTINGS_VERSION
      && settingsV5.checksum == computePersistedSettingsChecksum(settingsV5)) {
    timelapseIntervalSeconds = static_cast<uint8_t>(constrain(
        static_cast<int>(settingsV5.timelapseIntervalSeconds),
        TIMELAPSE_INTERVAL_MIN_SECONDS,
        TIMELAPSE_INTERVAL_MAX_SECONDS));
    stepDist = constrain(settingsV5.stepDist,
        TIMELAPSE_STEP_DIST_MIN_MS,
        TIMELAPSE_STEP_DIST_MAX_MS);
    timelapseSettleDwellMs = constrain(static_cast<unsigned long>(settingsV5.timelapseSettleDwellMs),
        TIMELAPSE_SETTLE_DWELL_MIN_MS,
        TIMELAPSE_SETTLE_DWELL_MAX_MS);
    rumbleMuted = ((settingsV5.flags & PERSISTED_SETTINGS_FLAG_RUMBLE_MUTED) != 0);
    homePoseValid = ((settingsV5.flags & PERSISTED_SETTINGS_FLAG_HOME_VALID) != 0);
    if (homePoseValid) {
      homePose.swing = settingsV5.homeSwing;
      homePose.lift = settingsV5.homeLift;
      homePose.pan = settingsV5.homePan;
      homePose.tilt = settingsV5.homeTilt;
      homePose.dwellMs = 0;
    }
    return true;
  }

  PersistedSettingsV4 settingsV4;
  EEPROM.get(EEPROM_SETTINGS_ADDRESS, settingsV4);

  if (settingsV4.magic == PERSISTED_SETTINGS_MAGIC
      && settingsV4.version == PERSISTED_SETTINGS_VERSION
      && settingsV4.checksum == computePersistedSettingsChecksum(settingsV4)) {
    timelapseIntervalSeconds = static_cast<uint8_t>(constrain(
        static_cast<int>(settingsV4.timelapseIntervalSeconds),
        TIMELAPSE_INTERVAL_MIN_SECONDS,
        TIMELAPSE_INTERVAL_MAX_SECONDS));
    stepDist = constrain(settingsV4.stepDist,
        TIMELAPSE_STEP_DIST_MIN_MS,
        TIMELAPSE_STEP_DIST_MAX_MS);
    timelapseSettleDwellMs = DEFAULT_TIMELAPSE_SETTLE_DWELL_MS;
    rumbleMuted = ((settingsV4.flags & PERSISTED_SETTINGS_FLAG_RUMBLE_MUTED) != 0);
    homePoseValid = ((settingsV4.flags & PERSISTED_SETTINGS_FLAG_HOME_VALID) != 0);
    if (homePoseValid) {
      homePose.swing = settingsV4.homeSwing;
      homePose.lift = settingsV4.homeLift;
      homePose.pan = settingsV4.homePan;
      homePose.tilt = settingsV4.homeTilt;
      homePose.dwellMs = 0;
    }
    return true;
  }

  PersistedSettingsV3 settingsV3;
  EEPROM.get(EEPROM_SETTINGS_ADDRESS, settingsV3);

  if (settingsV3.magic == PERSISTED_SETTINGS_MAGIC
      && settingsV3.version == 3
      && settingsV3.checksum == computePersistedSettingsChecksum(settingsV3)) {
    timelapseIntervalSeconds = static_cast<uint8_t>(constrain(
        static_cast<int>(settingsV3.timelapseIntervalSeconds),
        TIMELAPSE_INTERVAL_MIN_SECONDS,
        TIMELAPSE_INTERVAL_MAX_SECONDS));
    stepDist = constrain(settingsV3.stepDist,
        TIMELAPSE_STEP_DIST_MIN_MS,
        TIMELAPSE_STEP_DIST_MAX_MS);
    timelapseSettleDwellMs = DEFAULT_TIMELAPSE_SETTLE_DWELL_MS;
    rumbleMuted = ((settingsV3.flags & PERSISTED_SETTINGS_FLAG_RUMBLE_MUTED) != 0);
    homePoseValid = false;
    return true;
  }

  PersistedSettingsV2 settingsV2;
  EEPROM.get(EEPROM_SETTINGS_ADDRESS, settingsV2);

  if (settingsV2.magic == PERSISTED_SETTINGS_MAGIC
      && settingsV2.version == 2
      && settingsV2.checksum == computePersistedSettingsChecksum(settingsV2)) {
    timelapseIntervalSeconds = static_cast<uint8_t>(constrain(
        static_cast<int>(settingsV2.timelapseIntervalSeconds),
        TIMELAPSE_INTERVAL_MIN_SECONDS,
        TIMELAPSE_INTERVAL_MAX_SECONDS));
    stepDist = constrain(settingsV2.stepDist,
        TIMELAPSE_STEP_DIST_MIN_MS,
        TIMELAPSE_STEP_DIST_MAX_MS);
    timelapseSettleDwellMs = DEFAULT_TIMELAPSE_SETTLE_DWELL_MS;
    homePoseValid = false;
    return true;
  }

  PersistedSettingsV1 settingsV1;
  EEPROM.get(EEPROM_SETTINGS_ADDRESS, settingsV1);
  if (settingsV1.magic != PERSISTED_SETTINGS_MAGIC
      || settingsV1.version != 1
      || settingsV1.checksum != computePersistedSettingsChecksum(settingsV1)) {
    return false;
  }

  timelapseIntervalSeconds = static_cast<uint8_t>(constrain(
      static_cast<int>(settingsV1.timelapseIntervalSeconds),
      TIMELAPSE_INTERVAL_MIN_SECONDS,
      TIMELAPSE_INTERVAL_MAX_SECONDS));
  timelapseSettleDwellMs = DEFAULT_TIMELAPSE_SETTLE_DWELL_MS;
  homePoseValid = false;
  return true;
}

void savePersistedSettings() {
  PersistedSettingsV5 settings = {};
  settings.magic = PERSISTED_SETTINGS_MAGIC;
  settings.version = PERSISTED_SETTINGS_VERSION;
  settings.timelapseIntervalSeconds = timelapseIntervalSeconds;
  settings.stepDist = stepDist;
  settings.timelapseSettleDwellMs = static_cast<uint16_t>(constrain(timelapseSettleDwellMs,
      TIMELAPSE_SETTLE_DWELL_MIN_MS,
      TIMELAPSE_SETTLE_DWELL_MAX_MS));
  settings.flags = 0;
  if (rumbleMuted) {
    settings.flags |= PERSISTED_SETTINGS_FLAG_RUMBLE_MUTED;
  }
  if (homePoseValid) {
    settings.flags |= PERSISTED_SETTINGS_FLAG_HOME_VALID;
    settings.homeSwing = homePose.swing;
    settings.homeLift = homePose.lift;
    settings.homePan = homePose.pan;
    settings.homeTilt = homePose.tilt;
  }
  settings.checksum = computePersistedSettingsChecksum(settings);
  writePersistedSettings(settings);
}

void restorePersistedSettingDefaults() {
  timelapseIntervalSeconds = DEFAULT_TIMELAPSE_INTERVAL_SECONDS;
  stepDist = DEFAULT_TIMELAPSE_STEP_DIST_MS;
  timelapseSettleDwellMs = DEFAULT_TIMELAPSE_SETTLE_DWELL_MS;
  rumbleMuted = false;
  homePoseValid = false;
  homeReturnActive = false;
  updateIntervalMs();
  enforceTimelapseTimingBudget(false);
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
      Serial.println(F("Timelapse interval adjustment blocked: auto mode active."));
    }
  }

  if (intervalAdjustDownComboRawActive && !lastIntervalAdjustDownComboActive) {
    if (adjustmentAllowed) {
      adjustIntervalSeconds(-1);
    } else {
      startLockoutDeniedRumbleFeedback();
      Serial.println(F("Timelapse interval adjustment blocked: auto mode active."));
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

// START + PAD_RIGHT increases stepDist.
// START + PAD_LEFT decreases stepDist.
// stepDist changes by 10 ms per press.
// This is only active while no auto mode is running so it does not conflict
// with active timelapse or bounce motion.
bool handleTimelapseStepDistAdjustment() {
  bool adjustmentAllowed = !isAutoModeActive();
  bool stepDistAdjustUpComboRawActive = ps2x.Button(PSB_START) && ps2x.Button(PSB_PAD_RIGHT);
  bool stepDistAdjustDownComboRawActive = ps2x.Button(PSB_START) && ps2x.Button(PSB_PAD_LEFT);

  if (stepDistAdjustUpComboRawActive && !lastStepDistAdjustUpComboActive) {
    if (adjustmentAllowed) {
      adjustStepDist(TIMELAPSE_STEP_DIST_ADJUST_INCREMENT_MS);
    } else {
      startLockoutDeniedRumbleFeedback();
      Serial.println(F("Timelapse stepDist adjustment blocked: auto mode active."));
    }
  }

  if (stepDistAdjustDownComboRawActive && !lastStepDistAdjustDownComboActive) {
    if (adjustmentAllowed) {
      adjustStepDist(-TIMELAPSE_STEP_DIST_ADJUST_INCREMENT_MS);
    } else {
      startLockoutDeniedRumbleFeedback();
      Serial.println(F("Timelapse stepDist adjustment blocked: auto mode active."));
    }
  }

  lastStepDistAdjustUpComboActive = stepDistAdjustUpComboRawActive;
  lastStepDistAdjustDownComboActive = stepDistAdjustDownComboRawActive;

  bool stepDistAdjustComboHandled = adjustmentAllowed && (stepDistAdjustUpComboRawActive || stepDistAdjustDownComboRawActive);

  if (stepDistAdjustUpComboRawActive || stepDistAdjustDownComboRawActive) {
    suppressNextStartRelease = true;
  }

  if (stepDistAdjustComboHandled) {
    stopAllMotors();
    return true;
  }

  return false;
}

// START + SELECT + PAD_RIGHT increases settle dwell.
// START + SELECT + PAD_LEFT decreases settle dwell.
// Dwell changes by 100 ms per press.
bool handleTimelapseSettleDwellAdjustment() {
  bool adjustmentAllowed = !isAutoModeActive();
  bool dwellAdjustUpComboRawActive = ps2x.Button(PSB_START) && ps2x.Button(PSB_SELECT) && ps2x.Button(PSB_PAD_RIGHT);
  bool dwellAdjustDownComboRawActive = ps2x.Button(PSB_START) && ps2x.Button(PSB_SELECT) && ps2x.Button(PSB_PAD_LEFT);

  if (dwellAdjustUpComboRawActive && !lastTimelapseSettleAdjustUpComboActive) {
    if (adjustmentAllowed) {
      adjustTimelapseSettleDwell(static_cast<int>(TIMELAPSE_SETTLE_DWELL_ADJUST_INCREMENT_MS));
    } else {
      startLockoutDeniedRumbleFeedback();
      Serial.println(F("Timelapse dwell adjustment blocked: auto mode active."));
    }
  }

  if (dwellAdjustDownComboRawActive && !lastTimelapseSettleAdjustDownComboActive) {
    if (adjustmentAllowed) {
      adjustTimelapseSettleDwell(-static_cast<int>(TIMELAPSE_SETTLE_DWELL_ADJUST_INCREMENT_MS));
    } else {
      startLockoutDeniedRumbleFeedback();
      Serial.println(F("Timelapse dwell adjustment blocked: auto mode active."));
    }
  }

  lastTimelapseSettleAdjustUpComboActive = dwellAdjustUpComboRawActive;
  lastTimelapseSettleAdjustDownComboActive = dwellAdjustDownComboRawActive;

  bool dwellAdjustComboHandled = adjustmentAllowed && (dwellAdjustUpComboRawActive || dwellAdjustDownComboRawActive);

  if (dwellAdjustUpComboRawActive || dwellAdjustDownComboRawActive) {
    suppressNextStartRelease = true;
    suppressNextSelectRelease = true;
  }

  if (dwellAdjustComboHandled) {
    stopAllMotors();
    return true;
  }

  return false;
}

// START + L2 decreases max speed stage. START + R2 increases max speed stage.
// Speed stage changes by 1 per press (1 is slowest, 4 is fastest).
bool handleTimelapseMaxSpeedStageAdjustment() {
  bool adjustmentAllowed = !isAutoModeActive();
  bool stageAdjustUpComboRawActive = ps2x.Button(PSB_START) && ps2x.Button(PSB_R2);
  bool stageAdjustDownComboRawActive = ps2x.Button(PSB_START) && ps2x.Button(PSB_L2);

  if (stageAdjustUpComboRawActive && !lastTimelapseMaxSpeedStageAdjustUpComboActive) {
    if (adjustmentAllowed) {
      if (timelapseMaxSpeedStage < TIMELAPSE_MAX_SPEED_STAGE_MAX) {
        timelapseMaxSpeedStage++;
        Serial.print(F("Timelapse max speed stage += 1 -> "));
        Serial.println(timelapseMaxSpeedStage);
        startFeedbackRumble(1, FEEDBACK_RUMBLE_ON_MS, FEEDBACK_RUMBLE_TOTAL_MS);
        char statusBuf[32];
        snprintf(statusBuf, sizeof(statusBuf), "TL_MAX_STAGE:%u", timelapseMaxSpeedStage);
        broadcastStatus(statusBuf);
      } else {
        startLimitReachedRumbleFeedback();
        Serial.println(F("Timelapse max speed stage already at maximum (4)."));
      }
    } else {
      startLockoutDeniedRumbleFeedback();
      Serial.println(F("Timelapse max speed stage adjustment blocked: auto mode active."));
    }
  }

  if (stageAdjustDownComboRawActive && !lastTimelapseMaxSpeedStageAdjustDownComboActive) {
    if (adjustmentAllowed) {
      if (timelapseMaxSpeedStage > TIMELAPSE_MAX_SPEED_STAGE_MIN) {
        timelapseMaxSpeedStage--;
        Serial.print(F("Timelapse max speed stage -= 1 -> "));
        Serial.println(timelapseMaxSpeedStage);
        startFeedbackRumble(1, FEEDBACK_RUMBLE_ON_MS, FEEDBACK_RUMBLE_TOTAL_MS);
        char statusBuf[32];
        snprintf(statusBuf, sizeof(statusBuf), "TL_MAX_STAGE:%u", timelapseMaxSpeedStage);
        broadcastStatus(statusBuf);
      } else {
        startLimitReachedRumbleFeedback();
        Serial.println(F("Timelapse max speed stage already at minimum (1)."));
      }
    } else {
      startLockoutDeniedRumbleFeedback();
      Serial.println(F("Timelapse max speed stage adjustment blocked: auto mode active."));
    }
  }

  lastTimelapseMaxSpeedStageAdjustUpComboActive = stageAdjustUpComboRawActive;
  lastTimelapseMaxSpeedStageAdjustDownComboActive = stageAdjustDownComboRawActive;

  bool stageAdjustComboHandled = adjustmentAllowed && (stageAdjustUpComboRawActive || stageAdjustDownComboRawActive);

  if (stageAdjustUpComboRawActive || stageAdjustDownComboRawActive) {
    suppressNextStartRelease = true;
  }

  if (stageAdjustComboHandled) {
    stopAllMotors();
    return true;
  }

  return false;
}

// START + SELECT + R1 toggles anti-backlash preload on/off.
bool handleTimelapseAntiBacklashToggle() {
  bool adjustmentAllowed = !isAutoModeActive();
  bool antiBacklashToggleComboRawActive = ps2x.Button(PSB_START) && ps2x.Button(PSB_SELECT) && ps2x.Button(PSB_R1);

  if (antiBacklashToggleComboRawActive && !lastTimelapseAntiBacklashToggleComboActive) {
    if (adjustmentAllowed) {
      timelapseAntiBacklashEnabled = !timelapseAntiBacklashEnabled;
      Serial.print(F("Timelapse anti-backlash "));
      Serial.println(timelapseAntiBacklashEnabled ? F("enabled.") : F("disabled."));
      startFeedbackRumble(1, FEEDBACK_RUMBLE_ON_MS, FEEDBACK_RUMBLE_TOTAL_MS);
      broadcastStatus(timelapseAntiBacklashEnabled ? "TL_BACKLASH:ON" : "TL_BACKLASH:OFF");
    } else {
      startLockoutDeniedRumbleFeedback();
      Serial.println(F("Timelapse anti-backlash toggle blocked: auto mode active."));
    }
  }

  lastTimelapseAntiBacklashToggleComboActive = antiBacklashToggleComboRawActive;

  if (antiBacklashToggleComboRawActive) {
    suppressNextStartRelease = true;
    suppressNextSelectRelease = true;
  }

  bool antiBacklashToggleComboHandled = adjustmentAllowed && antiBacklashToggleComboRawActive;
  if (antiBacklashToggleComboHandled) {
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
    savePersistedSettings();

    Serial.print(F("Controller rumble "));
    Serial.println(rumbleMuted ? "muted." : "unmuted.");

    broadcastStatus(rumbleMuted ? "RUMBLE_MUTE:ON" : "RUMBLE_MUTE:OFF");

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
      Serial.println(F("Settings replay blocked: auto mode active."));
    }
  }

  lastSettingsReplayComboActive = settingsReplayComboActive;
  return settingsReplayComboActive;
}

bool handlePersistedSettingsReset(unsigned long now) {
  bool resetAllowed = !droneMode && !isAutoModeActive();
  bool resetComboActive = ps2x.Button(PSB_START) && ps2x.Button(PSB_SELECT) && ps2x.Button(PSB_CIRCLE);

  if (!resetComboActive) {
    persistedSettingsResetComboActive = false;
    persistedSettingsResetStartMs = 0;
    persistedSettingsResetLatched = false;
    return false;
  }

  if (!persistedSettingsResetComboActive) {
    persistedSettingsResetComboActive = true;
    persistedSettingsResetStartMs = now;
  }

  if (!resetAllowed) {
    if (!persistedSettingsResetLatched) {
      persistedSettingsResetLatched = true;
      startLockoutDeniedRumbleFeedback();
      Serial.println(F("Persisted settings reset blocked: auto mode or drone mode active."));
    }
    stopAllMotors();
    return true;
  }

  if (!persistedSettingsResetLatched
      && now - persistedSettingsResetStartMs >= PERSISTED_SETTINGS_RESET_HOLD_MS) {
    restorePersistedSettingDefaults();
    savePersistedSettings();
    stopIntervalRumbleFeedback();
    suppressNextSelectRelease = true;
    suppressNextStartRelease = true;
    persistedSettingsResetLatched = true;
    startFeedbackRumble(2, FEEDBACK_RUMBLE_ON_MS, FEEDBACK_RUMBLE_TOTAL_MS);
    Serial.println(F("Persisted settings reset: timelapse interval, stepDist, and rumble mute restored to defaults."));
  }

  stopAllMotors();
  return true;
}

void enterSettingsMode() {
  settingsMode = true;
  stopAllMotors();
  sendToDisplayESP("SETTINGS_NAV:OPEN");
  sendToRGBESP("SETTINGS:OPEN");
  Serial.println(F("Settings mode activated."));
}

void exitSettingsMode() {
  settingsMode = false;
  lastL3ButtonState = ps2x.Button(PSB_L3);
  lastStartButtonState = ps2x.Button(PSB_START);
  lastSelectButtonState = ps2x.Button(PSB_SELECT);
  sendToDisplayESP("SETTINGS_NAV:CLOSE");
  sendToRGBESP("SETTINGS:CLOSE");
  Serial.println(F("Settings mode deactivated."));
}

void handleSettingsModeToggle() {
  static bool settingsToggleHoldActive = false;
  static bool settingsToggleLatched = false;
  static unsigned long settingsToggleHoldStartMs = 0;

  // Settings navigation is manual/idle only. Do not allow entering settings
  // while bounce or timelapse auto motion is running.
  if (timelapseMode != 0 || bounce != 0) {
    settingsToggleHoldActive = false;
    settingsToggleLatched = false;
    settingsToggleHoldStartMs = 0;
    return;
  }

  bool settingsToggleComboActive = ps2x.Button(PSB_START)
      && ps2x.Button(PSB_SELECT)
      && !ps2x.Button(PSB_TRIANGLE)
      && !ps2x.Button(PSB_CIRCLE)
      && !ps2x.Button(PSB_CROSS)
      && !ps2x.Button(PSB_SQUARE);

  if (!settingsToggleComboActive) {
    settingsToggleHoldActive = false;
    settingsToggleLatched = false;
    settingsToggleHoldStartMs = 0;
    return;
  }

  unsigned long now = millis();
  if (!settingsToggleHoldActive) {
    settingsToggleHoldActive = true;
    settingsToggleHoldStartMs = now;
  }

  if (settingsToggleLatched || (now - settingsToggleHoldStartMs < SETTINGS_TOGGLE_HOLD_MS)) {
    return;
  }

  settingsToggleLatched = true;
  suppressNextSelectRelease = true;
  suppressNextStartRelease = true;

  if (settingsMode) {
    exitSettingsMode();
  } else {
    enterSettingsMode();
  }
}

void handleSettingsInput() {
  static bool lastDpadUp = false;
  static bool lastDpadDown = false;
  static bool lastCircle = false;
  static bool lastCross = false;
  static bool lastL1 = false;
  static bool lastR1 = false;
  static unsigned long dpadUpHoldStart = 0;
  static unsigned long dpadDownHoldStart = 0;
  static unsigned long dpadUpLastRepeat = 0;
  static unsigned long dpadDownLastRepeat = 0;
  static bool lastDpadRight = false;
  static bool lastDpadLeft = false;
  static unsigned long dpadRightHoldStart = 0;
  static unsigned long dpadLeftHoldStart = 0;
  static unsigned long dpadRightLastRepeat = 0;
  static unsigned long dpadLeftLastRepeat = 0;
  static const unsigned long REPEAT_DELAY = 400;
  static const unsigned long REPEAT_INTERVAL = 150;

  bool dpadUp = ps2x.Button(PSB_PAD_UP);
  bool dpadDown = ps2x.Button(PSB_PAD_DOWN);
  bool dpadRight = ps2x.Button(PSB_PAD_RIGHT);
  bool dpadLeft = ps2x.Button(PSB_PAD_LEFT);
  bool circle = ps2x.Button(PSB_CIRCLE);
  bool cross = ps2x.Button(PSB_CROSS);
  bool l1 = ps2x.Button(PSB_L1);
  bool r1 = ps2x.Button(PSB_R1);
  unsigned long now = millis();

  if (dpadUp && !lastDpadUp) {
    sendToDisplayESP("SETTINGS_NAV:UP");
    dpadUpHoldStart = now;
    dpadUpLastRepeat = now;
  } else if (dpadUp && (now - dpadUpHoldStart >= REPEAT_DELAY)
             && (now - dpadUpLastRepeat >= REPEAT_INTERVAL)) {
    sendToDisplayESP("SETTINGS_NAV:UP");
    dpadUpLastRepeat = now;
  }

  if (dpadDown && !lastDpadDown) {
    sendToDisplayESP("SETTINGS_NAV:DOWN");
    dpadDownHoldStart = now;
    dpadDownLastRepeat = now;
  } else if (dpadDown && (now - dpadDownHoldStart >= REPEAT_DELAY)
             && (now - dpadDownLastRepeat >= REPEAT_INTERVAL)) {
    sendToDisplayESP("SETTINGS_NAV:DOWN");
    dpadDownLastRepeat = now;
  }

  if (circle && !lastCircle) {
    sendToDisplayESP("SETTINGS_NAV:SELECT");
  }
  if (cross && !lastCross) {
    sendToDisplayESP("SETTINGS_NAV:BACK");
  }
  if (l1 && !lastL1) {
    sendToDisplayESP("SETTINGS_NAV:GROUP_PREV");
  }
  if (r1 && !lastR1) {
    sendToDisplayESP("SETTINGS_NAV:GROUP_NEXT");
  }

  if (dpadRight && !lastDpadRight) {
    sendToDisplayESP("SETTINGS_NAV:RIGHT");
    dpadRightHoldStart = now;
    dpadRightLastRepeat = now;
  } else if (dpadRight && (now - dpadRightHoldStart >= REPEAT_DELAY)
             && (now - dpadRightLastRepeat >= REPEAT_INTERVAL)) {
    sendToDisplayESP("SETTINGS_NAV:RIGHT");
    dpadRightLastRepeat = now;
  }

  if (dpadLeft && !lastDpadLeft) {
    sendToDisplayESP("SETTINGS_NAV:LEFT");
    dpadLeftHoldStart = now;
    dpadLeftLastRepeat = now;
  } else if (dpadLeft && (now - dpadLeftHoldStart >= REPEAT_DELAY)
             && (now - dpadLeftLastRepeat >= REPEAT_INTERVAL)) {
    sendToDisplayESP("SETTINGS_NAV:LEFT");
    dpadLeftLastRepeat = now;
  }

  lastDpadUp = dpadUp;
  lastDpadDown = dpadDown;
  lastDpadRight = dpadRight;
  lastDpadLeft = dpadLeft;
  lastCircle = circle;
  lastCross = cross;
  lastL1 = l1;
  lastR1 = r1;
}

void handleDroneModeToggle() {
  if (settingsMode) return;

  bool currentR3ButtonState = ps2x.Button(PSB_R3);
  unsigned long now = millis();
  if (!lastR3ButtonState && currentR3ButtonState) {
    r3PressStartMs = now;
  }
  bool r3JustReleased = lastR3ButtonState && !currentR3ButtonState;
  lastR3ButtonState = currentR3ButtonState;
  
  if (!r3JustReleased) {
    return;
  }

  if (droneMode) {
    unsigned long heldMs = (now >= r3PressStartMs) ? (now - r3PressStartMs) : 0;
    if (heldMs >= DRONE_MODE_EXIT_HOLD_MS) {
      exitDroneMode();
    }
  } else {
    enterDroneMode();
  }
}

void logEmergencyStopEvent() {
  Serial.print(F("EMERGENCY STOP | timelapseMode="));
  Serial.print(timelapseMode);
  Serial.print(F(" bounce="));
  Serial.print(bounce);
  Serial.print(F(" intervalSeconds="));
  Serial.print(timelapseIntervalSeconds);
  Serial.print(F(" stepDistMs="));
  Serial.println(stepDist);
}

// Emergency stop combo: L1 + L2 + R1 + R2
// Immediately stops all motor outputs and clears active auto modes.
bool handleEmergencyStop() {
  bool emergencyStopComboActive = ps2x.Button(PSB_L1) && ps2x.Button(PSB_L2) && ps2x.Button(PSB_R1) && ps2x.Button(PSB_R2);

  if (!emergencyStopComboActive) {
    if (lastEmergencyStopComboActive) {
      Serial.println(F("EMERGENCY STOP RELEASED | controls re-enabled"));
      broadcastStatus("EMERGENCY_STOP:RELEASED");
      startEmergencyReleaseRumbleFeedback();
      stopIntervalRumbleFeedback();
    }
    lastEmergencyStopComboActive = false;
    return false;
  }

  if (!lastEmergencyStopComboActive) {
    logEmergencyStopEvent();
    broadcastStatus("EMERGENCY_STOP:ACTIVE");
    resetTimelapseState();
    resetBounceState();
    settingsMode = false;
    if (droneMode) {
      resetFlowlapseSession(false);
      Serial.println(F("Flowlapse: canceled by emergency stop."));
    }
    stopIntervalRumbleFeedback();
  }

  stopAllMotors();
  lastEmergencyStopComboActive = true;
  return true;
}

const char* getTimelapseModeLabel(uint8_t mode) {
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

// Gradually increase speed by pulsing speed-up pins for all active axes in the timelapse mode
void applyTimelapseSpeedIncrease(uint8_t mode) {
  switch (mode) {
    case 1:  // swing left, boom down
    case 2:  // swing left, boom up
    case 3:  // swing right, boom up
    case 4:  // swing right, boom down
      digitalWrite(swingSpeedUp, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(swingSpeedUp, LOW);
      digitalWrite(panSpeedUp, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(panSpeedUp, LOW);
      digitalWrite(liftSpeedUp, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(liftSpeedUp, LOW);
      digitalWrite(tiltSpeedUp, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(tiltSpeedUp, LOW);
      break;
    case 5:  // swing left only
      digitalWrite(swingSpeedUp, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(swingSpeedUp, LOW);
      digitalWrite(panSpeedUp, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(panSpeedUp, LOW);
      break;
    case 6:  // boom up only
      digitalWrite(liftSpeedUp, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(liftSpeedUp, LOW);
      digitalWrite(tiltSpeedUp, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(tiltSpeedUp, LOW);
      break;
    case 7:  // swing right only
      digitalWrite(swingSpeedUp, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(swingSpeedUp, LOW);
      digitalWrite(panSpeedUp, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(panSpeedUp, LOW);
      break;
    case 8:  // boom down only
      digitalWrite(liftSpeedUp, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(liftSpeedUp, LOW);
      digitalWrite(tiltSpeedUp, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(tiltSpeedUp, LOW);
      break;
  }
}

// Gradually decrease speed by pulsing speed-down pins for all active axes in the timelapse mode
void applyTimelapseSpeedDecrease(uint8_t mode) {
  switch (mode) {
    case 1:  // swing left, boom down
      digitalWrite(swingSpeedDown, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(swingSpeedDown, LOW);
      digitalWrite(panSpeedUpOnly, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(panSpeedUpOnly, LOW);
      digitalWrite(liftSpeedDown, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(liftSpeedDown, LOW);
      digitalWrite(tiltSpeedDownOnly, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(tiltSpeedDownOnly, LOW);
      break;
    case 2:  // swing left, boom up
      digitalWrite(swingSpeedDown, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(swingSpeedDown, LOW);
      digitalWrite(panSpeedUpOnly, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(panSpeedUpOnly, LOW);
      digitalWrite(liftSpeedDown, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(liftSpeedDown, LOW);
      digitalWrite(tiltSpeedDownOnly, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(tiltSpeedDownOnly, LOW);
      break;
    case 3:  // swing right, boom up
      digitalWrite(swingSpeedDown, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(swingSpeedDown, LOW);
      digitalWrite(panSpeedDownOnly, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(panSpeedDownOnly, LOW);
      digitalWrite(liftSpeedDown, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(liftSpeedDown, LOW);
      digitalWrite(tiltSpeedUpOnly, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(tiltSpeedUpOnly, LOW);
      break;
    case 4:  // swing right, boom down
      digitalWrite(swingSpeedDown, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(swingSpeedDown, LOW);
      digitalWrite(panSpeedDownOnly, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(panSpeedDownOnly, LOW);
      digitalWrite(liftSpeedDown, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(liftSpeedDown, LOW);
      digitalWrite(tiltSpeedUpOnly, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(tiltSpeedUpOnly, LOW);
      break;
    case 5:  // swing left only
      digitalWrite(swingSpeedDown, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(swingSpeedDown, LOW);
      digitalWrite(panSpeedUpOnly, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(panSpeedUpOnly, LOW);
      break;
    case 6:  // boom up only
      digitalWrite(liftSpeedDown, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(liftSpeedDown, LOW);
      digitalWrite(tiltSpeedUpOnly, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(tiltSpeedUpOnly, LOW);
      break;
    case 7:  // swing right only
      digitalWrite(swingSpeedDown, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(swingSpeedDown, LOW);
      digitalWrite(panSpeedDownOnly, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(panSpeedDownOnly, LOW);
      break;
    case 8:  // boom down only
      digitalWrite(liftSpeedDown, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(liftSpeedDown, LOW);
      digitalWrite(tiltSpeedUpOnly, HIGH);
      delayMicroseconds(SPEED_STAGE_PULSE_HIGH_MS * 1000);
      digitalWrite(tiltSpeedUpOnly, LOW);
      break;
  }
}

void applyTimelapseModeOutputs(uint8_t mode) {
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
  bool wasActive = (timelapseMode != 0);
  timelapseMode = 0;
  timelapsePhase = TIMELAPSE_PHASE_IDLE;
  timelapsePhaseStartMs = 0;
  timelapseCurrentSpeedStage = 0;
  digitalWrite(trigger, HIGH);
  stopAllMotors();
  if (wasActive) {
    normalizeMotionAxisSpeedStages(DEFAULT_AXIS_SPEED_STAGE);
  }
}

void resetBounceState() {
  bool wasActive = (bounce != 0);
  bounce = 0;
  stage = 0;
  bouncePhaseStartMs = 0;
  bounceMoveDurationMs = 0;
  bounceSwingTier = DEFAULT_AXIS_SPEED_STAGE;
  bouncePanTier = DEFAULT_AXIS_SPEED_STAGE;
  bounceLiftTier = DEFAULT_AXIS_SPEED_STAGE;
  bounceTiltTier = DEFAULT_AXIS_SPEED_STAGE;
  stopAllMotors();
  if (wasActive) {
    normalizeMotionAxisSpeedStages(DEFAULT_AXIS_SPEED_STAGE);
  }
}

// Returns 1-8 based on left-stick position, or 0 if no match.
// Used by both timelapse and bounce mode selection.
uint8_t stickPositionToMode(int stickX, int stickY) {
  if (stickX < STICK_MODE_LOW_THRESHOLD  && stickY > STICK_MODE_HIGH_THRESHOLD) return 1;
  if (stickX < STICK_MODE_LOW_THRESHOLD  && stickY < STICK_MODE_LOW_THRESHOLD)  return 2;
  if (stickX > STICK_MODE_HIGH_THRESHOLD && stickY < STICK_MODE_LOW_THRESHOLD)  return 3;
  if (stickX > STICK_MODE_HIGH_THRESHOLD && stickY > STICK_MODE_HIGH_THRESHOLD) return 4;
  if (stickX <= STICK_DEADBAND) return 5;
  if (stickY <= STICK_DEADBAND) return 6;
  if (stickX >= STICK_MAX - STICK_DEADBAND) return 7;
  if (stickY >= STICK_MAX - STICK_DEADBAND) return 8;
  return 0;
}

void updateTimelapseModeSelection() {
  bool currentSelectButtonState = ps2x.Button(PSB_SELECT);
  
  if (suppressNextSelectRelease) {
    // Detect release edge: was pressed, now not pressed
    if (lastSelectButtonState && !currentSelectButtonState) {
      suppressNextSelectRelease = false;
    }
    lastSelectButtonState = currentSelectButtonState;
    return;
  }

  // Detect release edge: was pressed (lastSelectButtonState=true), now not pressed (currentSelectButtonState=false)
  bool selectJustReleased = lastSelectButtonState && !currentSelectButtonState;
  
  if (timelapseMode != 0 || !selectJustReleased) {
    lastSelectButtonState = currentSelectButtonState;
    return;
  }
  
  timelapseMode = stickPositionToMode(leftStickXvalue, leftStickYvalue);
  if (timelapseMode == 0) {
    Serial.println(F("Timelapse: stick near center at SELECT release, no mode started."));
    lastSelectButtonState = currentSelectButtonState;
    return;
  }
  const char* tlLabel = getTimelapseModeLabel(timelapseMode);
  if (tlLabel != nullptr) {
    Serial.println(tlLabel);
  }
  
  lastSelectButtonState = currentSelectButtonState;
}

const char* getBounceModeSerialLabel(uint8_t mode) {
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

void broadcastModeStatusIfChanged() {
  StatusModeKind currentModeKind = STATUS_MODE_MANUAL;
  uint8_t currentModeVariant = 0;

  if (droneMode) {
    currentModeKind = STATUS_MODE_DRONE;
  } else if (timelapseMode != 0) {
    currentModeKind = STATUS_MODE_TIMELAPSE;
    currentModeVariant = timelapseMode;
  } else if (bounce != 0) {
    currentModeKind = STATUS_MODE_BOUNCE;
    currentModeVariant = bounce;
  }

  if (modeStatusInitialized
      && currentModeKind == lastBroadcastModeKind
      && currentModeVariant == lastBroadcastModeVariant) {
    return;
  }

  char modeLine[96];
  switch (currentModeKind) {
    case STATUS_MODE_DRONE:
      strcpy(modeLine, "MODE:DRONE");
      break;
    case STATUS_MODE_TIMELAPSE:
      snprintf(modeLine, sizeof(modeLine), "MODE:TIMELAPSE %u", currentModeVariant);
      break;
    case STATUS_MODE_BOUNCE:
      snprintf(modeLine, sizeof(modeLine), "MODE:BOUNCE %u", currentModeVariant);
      break;
    case STATUS_MODE_MANUAL:
    default:
      strcpy(modeLine, "MODE:MANUAL");
      break;
  }

  broadcastStatus(modeLine);
  modeStatusInitialized = true;
  lastBroadcastModeKind = currentModeKind;
  lastBroadcastModeVariant = currentModeVariant;
}

void updateBounceModeSelection() {
  bool currentStartButtonState = ps2x.Button(PSB_START);
  
  if (suppressNextStartRelease) {
    // Detect release edge: was pressed, now not pressed
    if (lastStartButtonState && !currentStartButtonState) {
      suppressNextStartRelease = false;
    }
    lastStartButtonState = currentStartButtonState;
    return;
  }

  // Detect release edge: was pressed (lastStartButtonState=true), now not pressed (currentStartButtonState=false)
  bool startJustReleased = lastStartButtonState && !currentStartButtonState;
  
  if (bounce != 0 || !startJustReleased) {
    lastStartButtonState = currentStartButtonState;
    return;
  }
  
  bounce = stickPositionToMode(leftStickXvalue, leftStickYvalue);
  if (bounce == 0) {
    Serial.println(F("Bounce: stick near center at START release, no mode started."));
    lastStartButtonState = currentStartButtonState;
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
void setBounceModeOutputs(uint8_t mode, bool towardEndpoint, uint8_t state) {
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

// Applies a speed tier to only the axes used by the given bounce mode.
void setBounceModeSpeedTier(uint8_t mode, uint8_t speedTier) {
  uint8_t clampedTier = static_cast<uint8_t>(constrain(static_cast<int>(speedTier),
      static_cast<int>(DRONE_SPEED_TIER_STOP),
      static_cast<int>(DRONE_SPEED_TIER_HIGH)));

  switch (mode) {
    case 1:
    case 2:
    case 3:
    case 4:
      pulseAxisTierToward(bounceSwingTier, clampedTier, swingSpeedUp, swingSpeedDown);
      pulseAxisTierToward(bouncePanTier, clampedTier, panSpeedUp, panSpeedDown);
      pulseAxisTierToward(bounceLiftTier, clampedTier, liftSpeedUp, liftSpeedDown);
      pulseAxisTierToward(bounceTiltTier, clampedTier, tiltSpeedUp, tiltSpeedDown);
      break;
    case 5:
    case 7:
      pulseAxisTierToward(bounceSwingTier, clampedTier, swingSpeedUp, swingSpeedDown);
      pulseAxisTierToward(bouncePanTier, clampedTier, panSpeedUp, panSpeedDown);
      break;
    case 6:
    case 8:
      pulseAxisTierToward(bounceLiftTier, clampedTier, liftSpeedUp, liftSpeedDown);
      pulseAxisTierToward(bounceTiltTier, clampedTier, tiltSpeedUp, tiltSpeedDown);
      break;
  }
}

uint8_t getBounceSpeedTierForHalf(unsigned long halfElapsedMs, unsigned long halfDurationMs) {
  if (halfDurationMs == 0) {
    return DRONE_SPEED_TIER_STOP;
  }

  unsigned long rampWindowMs = constrain(
      halfDurationMs / 4,
      BOUNCE_SPEED_RAMP_WINDOW_MIN_MS,
      BOUNCE_SPEED_RAMP_WINDOW_MAX_MS);

  unsigned long distanceToEdgeMs = min(halfElapsedMs, halfDurationMs - halfElapsedMs);

  if (distanceToEdgeMs <= BOUNCE_ENDPOINT_STOP_BAND_MS) {
    return DRONE_SPEED_TIER_STOP;
  }

  if (distanceToEdgeMs < rampWindowMs) {
    return DRONE_SPEED_TIER_MED;
  }

  return DRONE_SPEED_TIER_HIGH;
}

// Stops only the motors used by the given bounce mode.
// Called at the turnaround point between stage 0 and stage 1,
// and during direction switches inside stage 1.
// Modes 1-4 use all four axes; modes 5/7 use swing only; modes 6/8 use lift only.
void stopBounceModeOutputs(uint8_t mode) {
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
    Serial.print(F("Bounce endpoint rejected: minimum move time is "));
    Serial.print(BOUNCE_MIN_MOVE_DURATION_MS);
    Serial.println(F("ms."));
    return;
  }

  stopBounceModeOutputs(bounce);
  setBounceModeSpeedTier(bounce, DRONE_SPEED_TIER_STOP);
  bounceMoveDurationMs = measuredMoveDurationMs;
  bouncePhaseStartMs = now;
  stage = 1;
  startL3EndpointRumbleFeedback();
  emitControlIndicator("L3_BOUNCE_ENDPOINT");
  Serial.print(F("Bounce endpoint set: "));
  Serial.print(bounceMoveDurationMs);
  Serial.println(F("ms travel time"));
}

void handleBounceStage0(unsigned long now) {
  if (bounce == 0 || stage != 0) {
    return;
  }

  if (bouncePhaseStartMs == 0) {
    normalizeMotionAxisSpeedStages(DEFAULT_AXIS_SPEED_STAGE);
    bounceSwingTier = DEFAULT_AXIS_SPEED_STAGE;
    bouncePanTier = DEFAULT_AXIS_SPEED_STAGE;
    bounceLiftTier = DEFAULT_AXIS_SPEED_STAGE;
    bounceTiltTier = DEFAULT_AXIS_SPEED_STAGE;
    bouncePhaseStartMs = now;
  }

  setBounceModeSpeedTier(bounce, DRONE_SPEED_TIER_HIGH);
  setBounceModeOutputs(bounce, true, HIGH);

  // Note: L3 state already tracked in lastL3ButtonState for drone mode
  // Reuse same tracking since bounce and drone modes are mutually exclusive
  bool currentL3State = ps2x.Button(PSB_L3);
  if (lastL3ButtonState && !currentL3State) {
    advanceBounceToStage1();
  }
  lastL3ButtonState = currentL3State;
}

void handleBounceStage1(unsigned long now) {
  if (bounce == 0 || stage != 1) {
    return;
  }

  if (bounceMoveDurationMs == 0) {
    return;
  }

  unsigned long elapsed = (now - bouncePhaseStartMs) % (bounceMoveDurationMs * 2);
  bool movingTowardEndpoint = elapsed >= bounceMoveDurationMs;
  unsigned long halfElapsed = movingTowardEndpoint ? (elapsed - bounceMoveDurationMs) : elapsed;

  uint8_t targetSpeedTier = getBounceSpeedTierForHalf(halfElapsed, bounceMoveDurationMs);
  setBounceModeSpeedTier(bounce, targetSpeedTier);

  if (!movingTowardEndpoint) {
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
      digitalWrite(trigger, HIGH);
      timelapsePhase = TIMELAPSE_PHASE_TRIGGER_LOW;
      timelapsePhaseStartMs = now;
      break;
    case TIMELAPSE_PHASE_TRIGGER_LOW:
      if (now - timelapsePhaseStartMs >= static_cast<unsigned long>(timelapseIntervalMs / 2)) {
        digitalWrite(trigger, LOW);
        timelapsePhase = TIMELAPSE_PHASE_TRIGGER_HIGH;
        timelapsePhaseStartMs = now;
      }
      break;
    case TIMELAPSE_PHASE_TRIGGER_HIGH:
      if (now - timelapsePhaseStartMs >= static_cast<unsigned long>(timelapseIntervalMs / 2)) {
        applyTimelapseModeOutputs(timelapseMode);
        timelapsePhase = timelapseAntiBacklashEnabled
            ? TIMELAPSE_PHASE_MOVE_PRELOAD
            : TIMELAPSE_PHASE_MOVE_RAMP_UP;
        timelapsePhaseStartMs = now;
        timelapseCurrentSpeedStage = 0;
        if (timelapseAntiBacklashEnabled && timelapseMaxSpeedStage >= 1) {
          applyTimelapseSpeedIncrease(timelapseMode);
          timelapseCurrentSpeedStage = 1;
        }
      }
      break;
    case TIMELAPSE_PHASE_MOVE_PRELOAD:
      if (now - timelapsePhaseStartMs >= static_cast<unsigned long>(TIMELAPSE_ANTI_BACKLASH_DURATION_MS)) {
        timelapsePhase = TIMELAPSE_PHASE_MOVE_RAMP_UP;
        timelapsePhaseStartMs = now;
      }
      break;
    case TIMELAPSE_PHASE_MOVE_RAMP_UP:
      // Gradually increase speed stage toward timelapseMaxSpeedStage over TIMELAPSE_RAMP_UP_DURATION_MS
      {
        unsigned long elapsedMs = now - timelapsePhaseStartMs;
        uint8_t targetStage = (elapsedMs * timelapseMaxSpeedStage) / TIMELAPSE_RAMP_UP_DURATION_MS;
        if (targetStage > timelapseMaxSpeedStage) targetStage = timelapseMaxSpeedStage;
        
        if (targetStage > timelapseCurrentSpeedStage) {
          applyTimelapseSpeedIncrease(timelapseMode);
          timelapseCurrentSpeedStage = targetStage;
        }
        
        if (elapsedMs >= TIMELAPSE_RAMP_UP_DURATION_MS) {
          timelapsePhase = TIMELAPSE_PHASE_MOVE_CRUISE;
          timelapsePhaseStartMs = now;
          timelapseCurrentSpeedStage = timelapseMaxSpeedStage;
        }
      }
      break;
    case TIMELAPSE_PHASE_MOVE_CRUISE:
      // Maintain speed at timelapseMaxSpeedStage for cruise phase
      {
        unsigned long totalMoveDuration = TIMELAPSE_RAMP_UP_DURATION_MS + TIMELAPSE_RAMP_DOWN_DURATION_MS;
        unsigned long cruiseMinDuration = stepDist > totalMoveDuration ? stepDist - totalMoveDuration : TIMELAPSE_CRUISE_MIN_DURATION_MS;
        unsigned long elapsedMs = now - timelapsePhaseStartMs;
        
        if (elapsedMs >= cruiseMinDuration) {
          timelapsePhase = TIMELAPSE_PHASE_MOVE_RAMP_DOWN;
          timelapsePhaseStartMs = now;
        }
      }
      break;
    case TIMELAPSE_PHASE_MOVE_RAMP_DOWN:
      // Gradually decrease speed stage from timelapseMaxSpeedStage to 0 over TIMELAPSE_RAMP_DOWN_DURATION_MS
      {
        unsigned long elapsedMs = now - timelapsePhaseStartMs;
        uint8_t targetStage = (timelapseMaxSpeedStage * (TIMELAPSE_RAMP_DOWN_DURATION_MS - elapsedMs)) / TIMELAPSE_RAMP_DOWN_DURATION_MS;
        
        if (targetStage < timelapseCurrentSpeedStage) {
          applyTimelapseSpeedDecrease(timelapseMode);
          timelapseCurrentSpeedStage = targetStage;
        }
        
        if (elapsedMs >= TIMELAPSE_RAMP_DOWN_DURATION_MS) {
          stopAllMotors();
          timelapsePhase = TIMELAPSE_PHASE_SETTLE;
          timelapsePhaseStartMs = now;
          timelapseCurrentSpeedStage = 0;
        }
      }
      break;
    case TIMELAPSE_PHASE_SETTLE:
      if (now - timelapsePhaseStartMs >= timelapseSettleDwellMs) {
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
  Serial.print(F("Drone tuning | expo S/L/P/T="));
  Serial.print(DRONE_SWING_EXPO_PERCENT);
  Serial.print(F("/"));
  Serial.print(DRONE_LIFT_EXPO_PERCENT);
  Serial.print(F("/"));
  Serial.print(DRONE_PAN_EXPO_PERCENT);
  Serial.print(F("/"));
  Serial.println(DRONE_TILT_EXPO_PERCENT);

  Serial.print(F("Drone tuning | deadband S/L/P/T="));
  Serial.print(DRONE_SWING_DEADBAND);
  Serial.print(F("/"));
  Serial.print(DRONE_LIFT_DEADBAND);
  Serial.print(F("/"));
  Serial.print(DRONE_PAN_DEADBAND);
  Serial.print(F("/"));
  Serial.println(DRONE_TILT_DEADBAND);

  Serial.print(F("Drone tuning | max tier S/L/P/T="));
  Serial.print(DRONE_SWING_MAX_SPEED_TIER);
  Serial.print(F("/"));
  Serial.print(DRONE_LIFT_MAX_SPEED_TIER);
  Serial.print(F("/"));
  Serial.print(DRONE_PAN_MAX_SPEED_TIER);
  Serial.print(F("/"));
  Serial.println(DRONE_TILT_MAX_SPEED_TIER);

  Serial.print(F("Drone tuning | modifier precision/boost enabled="));
  Serial.print(DRONE_ENABLE_PRECISION_MODIFIER ? "Y" : "N");
  Serial.print(F("/"));
  Serial.println(DRONE_ENABLE_BOOST_MODIFIER ? "Y" : "N");

  Serial.print(F("Drone tuning | L2 priority over boost="));
  Serial.println(DRONE_L2_PRIORITY_OVER_BOOST ? "Y" : "N");

  Serial.print(F("Drone tuning | tier thresholds med/high="));
  Serial.print(DRONE_SPEED_TIER_MED_THRESHOLD);
  Serial.print(F("/"));
  Serial.println(DRONE_SPEED_TIER_HIGH_THRESHOLD);

  Serial.print(F("Drone tuning | idle timeout ms="));
  if (DRONE_IDLE_TIMEOUT_MS == 0) {
    Serial.println(F("disabled"));
  } else {
    Serial.println(DRONE_IDLE_TIMEOUT_MS);
  }

  Serial.print(F("Drone tuning | serial log enabled="));
  Serial.println(DRONE_SERIAL_LOG_ENABLED ? "Y" : "N");

  Serial.print(F("Flowlapse tuning | max points="));
  Serial.println(FLOWLAPSE_MAX_WAYPOINTS);

  Serial.print(F("Flowlapse tuning | min waypoint separation="));
  Serial.println(FLOWLAPSE_MIN_WAYPOINT_SEPARATION);

  Serial.print(F("Flowlapse tuning | preview speed scale="));
  Serial.println(FLOWLAPSE_PREVIEW_SPEED_SCALE, 2);

  Serial.print(F("Flowlapse tuning | max speed tier="));
  Serial.println(FLOWLAPSE_MAX_SPEED_TIER);

  Serial.print(F("Flowlapse tuning | ramp interval ms="));
  Serial.println(FLOWLAPSE_TIER_RAMP_INTERVAL_MS);

  Serial.print(F("Flowlapse tuning | rates manual/med/high="));
  Serial.print(FLOWLAPSE_MANUAL_TRACK_RATE_UNITS_PER_SEC);
  Serial.print(F("/"));
  Serial.print(FLOWLAPSE_MED_RATE_UNITS_PER_SEC);
  Serial.print(F("/"));
  Serial.println(FLOWLAPSE_HIGH_RATE_UNITS_PER_SEC);

  Serial.print(F("Flowlapse tuning | capture progress log ms="));
  Serial.println(FLOWLAPSE_CAPTURE_PROGRESS_LOG_MS);

  Serial.print(F("Flowlapse tuning | curved path mode="));
  Serial.println(FLOWLAPSE_CURVED_PATH_ENABLED ? "enabled" : "disabled");

  Serial.print(F("Flowlapse tuning | ease in/out="));
  Serial.println(FLOWLAPSE_EASE_IN_OUT_ENABLED ? "enabled" : "disabled");

  Serial.print(F("Flowlapse tuning | ease strength="));
  Serial.println(clampFlowlapse01(FLOWLAPSE_EASE_STRENGTH), 2);

  Serial.print(F("Flowlapse tuning | ping-pong loop="));
  Serial.println(FLOWLAPSE_PING_PONG_LOOP ? "enabled" : "disabled");

  Serial.print(F("Flowlapse tuning | return-to-waypoint-1 on complete="));
  Serial.println(FLOWLAPSE_RETURN_TO_FIRST_WAYPOINT_ON_COMPLETE ? "enabled" : "disabled");

  Serial.print(F("Flowlapse tuning | arc-length sampling="));
  Serial.println(FLOWLAPSE_ARC_LENGTH_SAMPLING_ENABLED ? "enabled" : "disabled");

  Serial.print(F("Flowlapse tuning | arc-length quality preset="));
  if (FLOWLAPSE_ARC_LENGTH_QUALITY_PRESET == FLOWLAPSE_ARC_LENGTH_QUALITY_LOW) {
    Serial.println(F("low"));
  } else if (FLOWLAPSE_ARC_LENGTH_QUALITY_PRESET == FLOWLAPSE_ARC_LENGTH_QUALITY_HIGH) {
    Serial.println(F("high"));
  } else {
    Serial.println(F("med"));
  }

  Serial.print(F("Flowlapse tuning | arc-length LUT steps="));
  Serial.println(FLOWLAPSE_ARC_LENGTH_TABLE_STEPS);

  Serial.print(F("Flowlapse tuning | frame-count mode="));
  Serial.println(flowlapseFrameCountModeEnabled ? "enabled" : "disabled");

  Serial.print(F("Flowlapse tuning | frame-count target="));
  Serial.println(FLOWLAPSE_FRAME_COUNT_TARGET);

  Serial.print(F("Flowlapse tuning | frame-count auto-exit="));
  Serial.println(FLOWLAPSE_FRAMECOUNT_AUTO_EXIT ? "enabled" : "disabled");

  Serial.print(F("Flowlapse summary | ease="));
  Serial.print(FLOWLAPSE_EASE_IN_OUT_ENABLED ? "Y" : "N");
  Serial.print(F(" strength="));
  Serial.print(clampFlowlapse01(FLOWLAPSE_EASE_STRENGTH), 2);
  Serial.print(F(" loop="));
  Serial.print(FLOWLAPSE_LOOP_CAPTURE ? "Y" : "N");
  Serial.print(F(" frame="));
  Serial.print(flowlapseFrameCountModeEnabled ? "Y" : "N");
  Serial.print(F(" fexit="));
  Serial.print(FLOWLAPSE_FRAMECOUNT_AUTO_EXIT ? "Y" : "N");
  Serial.print(F(" arc="));
  Serial.print(FLOWLAPSE_ARC_LENGTH_SAMPLING_ENABLED ? "Y" : "N");
  Serial.print(F(" ping-pong="));
  Serial.print(FLOWLAPSE_PING_PONG_LOOP ? "Y" : "N");
  Serial.print(F(" return-home="));
  Serial.println(FLOWLAPSE_RETURN_TO_FIRST_WAYPOINT_ON_COMPLETE ? "Y" : "N");

  Serial.print(F("Flowlapse tuning | waypoint dwell ms="));
  if (flowlapseDwellMs == 0) {
    Serial.println(F("disabled"));
  } else {
    Serial.println(flowlapseDwellMs);
  }
}

void setup() {

  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial.println(F("BUILD: SPEED_LOG_V2 (WILL_TEST_SPEED_BUILD)"));
  bool persistedSettingsLoaded = loadPersistedSettings();
  updateIntervalMs();
  enforceTimelapseTimingBudget(false);

  const uint8_t outputPins[] = {
    LED_BUILTIN,
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

  stopAllMotors();
  applyStartupDefaultSpeedStages();
  Serial.print(F("Startup speed stage set to "));
  Serial.println(DEFAULT_AXIS_SPEED_STAGE);

  resetTimelapseState();

  delay(CONTROLLER_STARTUP_DELAY_MS);
  configureController();
  detectControllerType();
  if (error == 0) {
    const bool isDualShockAtBoot = (controllerType == 1 || controllerType == 3);
    if (isDualShockAtBoot) {
      broadcastStatus("CONTROLLER_OK:DualShock connected.");
    }
  }
  Serial.println(F("START + PAD_UP/DOWN adjusts timelapseIntervalSeconds."));
  Serial.println(F("START + PAD_RIGHT/LEFT adjusts stepDist by 10 ms."));
  Serial.println(F("START + SELECT + PAD_RIGHT/LEFT adjusts settle dwell by 100 ms."));
  Serial.println(F("START + SELECT + SQUARE toggles controller rumble mute."));
  Serial.println(F("Drone Mode: START + SELECT + TRIANGLE toggles Flowlapse frame-count mode."));
  Serial.println(F("Drone Mode: START+TRIANGLE=set home, START+SQUARE=go home, START+CIRCLE=clear home."));
  Serial.println(F("L1 + L2 + CIRCLE replays current settings as rumble patterns."));
  Serial.print(F("Boot settings: interval="));
  Serial.print(timelapseIntervalSeconds);
  Serial.print(F("s, stepDist="));
  Serial.print(stepDist);
  Serial.print(F("ms, dwell="));
  Serial.print(timelapseSettleDwellMs);
  Serial.print(F("ms, effectiveCycle="));
  Serial.print(timelapseIntervalMs + getTimelapseEffectiveMoveDurationMs() + timelapseSettleDwellMs);
  Serial.print(F("ms, budget="));
  Serial.print(timelapseIntervalMs);
  Serial.print(F("ms, rumbleMuted="));
  Serial.println(rumbleMuted ? F("YES") : F("NO"));
  Serial.print(F("Boot home pose valid="));
  Serial.println(homePoseValid ? F("YES") : F("NO"));
  Serial.println(persistedSettingsLoaded
      ? F("Persisted settings: timelapse interval restored from EEPROM.")
      : F("Persisted settings: defaults active."));
  printDroneTuningProfile();

  Serial.print(F("Boot axis reversal | swing="));
  Serial.print(digitalRead(DIP_SWITCH_1) == HIGH ? "Y" : "N");
  Serial.print(F(" pan="));
  Serial.print(digitalRead(DIP_SWITCH_2) == HIGH ? "Y" : "N");
  Serial.print(F(" lift="));
  Serial.print(digitalRead(DIP_SWITCH_3) == HIGH ? "Y" : "N");
  Serial.print(F(" tilt="));
  Serial.print(digitalRead(DIP_SWITCH_4) == HIGH ? "Y" : "N");
  Serial.print(F(" focus="));
  Serial.println(digitalRead(DIP_SWITCH_5) == HIGH ? "Y" : "N");
}

/* ---------- Display touchscreen SET command processing ---------- */

static char displayCmdBuf[48];
static uint8_t displayCmdLen = 0;

void processDisplayCommands() {
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (displayCmdLen == 0) continue;
      displayCmdBuf[displayCmdLen] = '\0';
      displayCmdLen = 0;

      if (strncmp(displayCmdBuf, "SET:TL_INT:", 11) == 0) {
        int val = atoi(displayCmdBuf + 11);
        if (val >= TIMELAPSE_INTERVAL_MIN_SECONDS && val <= TIMELAPSE_INTERVAL_MAX_SECONDS) {
          int delta = val - timelapseIntervalSeconds;
          if (delta != 0) {
            adjustIntervalSeconds(delta);
            Serial.print(F("Display SET timelapse interval = "));
            Serial.println(val);
          }
        }
      } else if (strncmp(displayCmdBuf, "SET:TL_STEP:", 12) == 0) {
        int val = atoi(displayCmdBuf + 12);
        if (val >= TIMELAPSE_STEP_DIST_MIN_MS && val <= TIMELAPSE_STEP_DIST_MAX_MS) {
          int delta = val - stepDist;
          if (delta != 0) {
            adjustStepDist(delta);
            Serial.print(F("Display SET timelapse stepDist = "));
            Serial.println(val);
          }
        }
      } else if (strncmp(displayCmdBuf, "SET:TL_DWELL:", 13) == 0) {
        long val = atol(displayCmdBuf + 13);
        if (val >= static_cast<long>(TIMELAPSE_SETTLE_DWELL_MIN_MS)
            && val <= static_cast<long>(TIMELAPSE_SETTLE_DWELL_MAX_MS)) {
          int delta = static_cast<int>(val - static_cast<long>(timelapseSettleDwellMs));
          if (delta != 0) {
            adjustTimelapseSettleDwell(delta);
            Serial.print(F("Display SET timelapse dwell = "));
            Serial.println(val);
          }
        }
      } else if (strncmp(displayCmdBuf, "SET:TL_MAX_STAGE:", 17) == 0) {
        int val = atoi(displayCmdBuf + 17);
        if (val >= TIMELAPSE_MAX_SPEED_STAGE_MIN && val <= TIMELAPSE_MAX_SPEED_STAGE_MAX) {
          if (val != static_cast<int>(timelapseMaxSpeedStage)) {
            timelapseMaxSpeedStage = static_cast<uint8_t>(val);
            Serial.print(F("Display SET timelapse max speed stage = "));
            Serial.println(val);
            char statusBuf[32];
            snprintf(statusBuf, sizeof(statusBuf), "TL_MAX_STAGE:%d", val);
            broadcastStatus(statusBuf);
          }
        }
      } else if (strncmp(displayCmdBuf, "SET:TL_BACKLASH:", 16) == 0) {
        bool newEnabled = (displayCmdBuf[16] == '1');
        if (newEnabled != timelapseAntiBacklashEnabled) {
          timelapseAntiBacklashEnabled = newEnabled;
          Serial.print(F("Display SET timelapse anti-backlash "));
          Serial.println(timelapseAntiBacklashEnabled ? F("ON") : F("OFF"));
          startFeedbackRumble(1, FEEDBACK_RUMBLE_ON_MS, FEEDBACK_RUMBLE_TOTAL_MS);
          broadcastStatus(timelapseAntiBacklashEnabled ? "TL_BACKLASH:ON" : "TL_BACKLASH:OFF");
        }
      } else if (strncmp(displayCmdBuf, "SET:RUMBLE_MUTE:", 16) == 0) {
        bool newMuted = (displayCmdBuf[16] == '1');
        if (newMuted != rumbleMuted) {
          rumbleMuted = newMuted;
          stopIntervalRumbleFeedback();
          savePersistedSettings();
          Serial.print(F("Display SET rumble "));
          Serial.println(rumbleMuted ? "muted." : "unmuted.");
          broadcastStatus(rumbleMuted ? "RUMBLE_MUTE:ON" : "RUMBLE_MUTE:OFF");
          if (!rumbleMuted) {
            startRumbleUnmuteFeedback();
          }
        }
      } else if (strncmp(displayCmdBuf, "SET:MTX_BRT:", 12) == 0) {
        sendToRGBESP(displayCmdBuf);
        Serial.print(F("Display "));
        Serial.println(displayCmdBuf);
      } else if (strncmp(displayCmdBuf, "SET:HOME_SET:", 13) == 0) {
        if (displayCmdBuf[13] == '1') {
          if (droneMode && !isFlowlapseBusyForHomeAction()) {
            setHomePoseFromCurrentPose(true);
            startFeedbackRumble(1, FLOWLAPSE_WAYPOINT_RUMBLE_ON_MS, FLOWLAPSE_WAYPOINT_RUMBLE_TOTAL_MS);
          } else {
            startLockoutDeniedRumbleFeedback();
            broadcastStatus("HOME:BLOCKED");
          }
        }
      } else if (strncmp(displayCmdBuf, "SET:HOME_GO:", 12) == 0) {
        if (displayCmdBuf[12] == '1') {
          if (droneMode) {
            startHomeReturn(millis());
          } else {
            startLockoutDeniedRumbleFeedback();
            broadcastStatus("HOME:BLOCKED");
          }
        }
      } else if (strncmp(displayCmdBuf, "SET:HOME_CLEAR:", 15) == 0) {
        if (displayCmdBuf[15] == '1') {
          if (droneMode && !isFlowlapseBusyForHomeAction()) {
            clearHomePose(true);
            startFeedbackRumble(2, FLOWLAPSE_WAYPOINT_RUMBLE_ON_MS, FLOWLAPSE_WAYPOINT_RUMBLE_TOTAL_MS);
          } else {
            startLockoutDeniedRumbleFeedback();
            broadcastStatus("HOME:BLOCKED");
          }
        }
      } else if (strncmp(displayCmdBuf, "SETTINGS_SAVED", 14) == 0) {
        if (!isRumbleFeedbackActive()) {
          startFeedbackRumble(1, FEEDBACK_RUMBLE_ON_MS, FEEDBACK_RUMBLE_TOTAL_MS);
        }
        Serial.println(F("Settings: save confirmed (rumble)."));
      } else if (strncmp(displayCmdBuf, "SETTINGS:OPEN", 13) == 0) {
        if (!settingsMode) {
          settingsMode = true;
          stopAllMotors();
          Serial.println(F("Settings mode activated (display)."));
        }
        sendToRGBESP(displayCmdBuf);
      } else if (strncmp(displayCmdBuf, "SETTINGS:CLOSE", 14) == 0) {
        if (settingsMode) {
          settingsMode = false;
          Serial.println(F("Settings mode deactivated (display)."));
        }
        sendToRGBESP(displayCmdBuf);
      }
      continue;
    }
    if (displayCmdLen < sizeof(displayCmdBuf) - 1) {
      displayCmdBuf[displayCmdLen++] = c;
    } else {
      displayCmdLen = 0;
    }
  }
}

void loop() {

  unsigned long now = millis();

  if (error != 0) {
    stopAllMotors();
    if (now - lastControllerRetryMs >= CONTROLLER_RETRY_INTERVAL_MS) {
      lastControllerRetryMs = now;
      Serial.println(F("Controller init failed. Retrying config..."));
      broadcastStatus("CONTROLLER_ERROR:No controller found, check wiring.");
      configureController();
      if (error == 0) {
        detectControllerType();
        broadcastStatus("CONTROLLER_OK:DualShock connected.");
      }
    }
    return;
  }

  if (controllerType == 2) {
    stopAllMotors();
    return;
  }

  const bool isDualShockType = (controllerType == 1 || controllerType == 3);
  if (!isDualShockType) {
    stopAllMotors();
    if (!unsupportedControllerWarningShown) {
      Serial.print(F("Unsupported controller type: "));
      Serial.println(controllerType);
      unsupportedControllerWarningShown = true;
    }
    return;
  }

  unsupportedControllerWarningShown = false;

  processDisplayCommands();

  if (now - lastControllerOkBroadcastMs >= CONTROLLER_OK_BROADCAST_INTERVAL_MS) {
    broadcastStatus("CONTROLLER_OK:DualShock connected.");
    lastControllerOkBroadcastMs = now;
  }

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
    emitLoopBypassReason("EMERGENCY_STOP");
    return;
  }

  handleDroneModeToggle();

  handleSettingsModeToggle();

  if (settingsMode) {
    handleSettingsInput();
    return;
  }

  if (!droneMode) {
    if (handlePersistedSettingsReset(now)) {
      emitLoopBypassReason("PERSISTED_SETTINGS_RESET");
      return;
    }

    if (handleTimelapseIntervalAdjustment()) {
      emitLoopBypassReason("TIMELAPSE_INTERVAL_ADJUST");
      return;
    }

    if (handleTimelapseStepDistAdjustment()) {
      emitLoopBypassReason("TIMELAPSE_STEPDIST_ADJUST");
      return;
    }

    if (handleTimelapseSettleDwellAdjustment()) {
      emitLoopBypassReason("TIMELAPSE_DWELL_ADJUST");
      return;
    }

    if (handleTimelapseMaxSpeedStageAdjustment()) {
      emitLoopBypassReason("TIMELAPSE_MAX_STAGE_ADJUST");
      return;
    }

    if (handleTimelapseAntiBacklashToggle()) {
      emitLoopBypassReason("TIMELAPSE_BACKLASH_TOGGLE");
      return;
    }

    if (handleRumbleMuteToggle()) {
      emitLoopBypassReason("RUMBLE_MUTE_TOGGLE");
      return;
    }

    if (handleSettingsReplay()) {
      emitLoopBypassReason("SETTINGS_REPLAY");
      return;
    }
  }

  if (droneMode) {
    float flowlapseDeltaSeconds = getFlowlapseDeltaSeconds(now);
    handleDroneFlowlapseWorkflow(now, flowlapseDeltaSeconds);
    broadcastModeStatusIfChanged();
    return;
  }

  handleAxisSpeedControl(PSB_R1, liftSpeedUp, tiltSpeedUp, "R1_LIFT_TILT_UP");
  handleAxisSpeedControl(PSB_R2, liftSpeedDown, tiltSpeedDown, "R2_LIFT_TILT_DOWN");
  handleAxisSpeedControl(PSB_L1, panSpeedUp, swingSpeedUp, "L1_PAN_SWING_UP");
  handleAxisSpeedControl(PSB_L2, panSpeedDown, swingSpeedDown, "L2_PAN_SWING_DOWN");
  logShoulderSpeedButtonEdges();

  if (bounce != 0) {
    updateBounceModeSelection();
    handleBounceStage0(now);
    handleBounceStage1(now);
    broadcastModeStatusIfChanged();
    return;
  }

  handleSwingMotionGroup();

  handlePanTrimAxis();
  handlePanAxis();

  handleLiftMotionGroup();

  handleTiltTrimAxis();
  handleTiltAxis();
  handleFocusAxis();

  updateTimelapseModeSelection();
  handleActiveTimelapseMode(now);

  updateBounceModeSelection();
  handleBounceStage0(now);
  handleBounceStage1(now);
  broadcastModeStatusIfChanged();
}

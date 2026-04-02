# MEGA + Nano Camera Rig

Arduino-based multi-controller camera rig using one Arduino Mega master and five Arduino Nano slave controllers.

## Repository Structure

- `MEGA__master/MEGA__master.ino` — main controller logic
- `NANO_slave_1_SWING/NANO_slave_1_SWING.ino` — swing axis slave
- `NANO_slave_2_PAN/NANO_slave_2_PAN.ino` — pan axis slave
- `NANO_slave_3_LIFT/NANO_slave_3_LIFT.ino` — lift axis slave
- `NANO_slave_4_TILT/NANO_slave_4_TILT.ino` — tilt axis slave
- `NANO_slave_5_FOCUS/NANO_slave_5_FOCUS.ino` — focus axis slave

## Notes

- Each folder contains a standalone `.ino` sketch for that board.
- Upload the corresponding sketch to each target board (Mega master or Nano slave).
- Keep serial/communication settings synchronized between master and all slaves.

## Dependencies

- Required Arduino library: `PS2X_lib` (for `#include <PS2X_lib.h>` in the Mega sketch)
- Source: https://github.com/madsci1016/Arduino-PS2X.git
- Vendored copy in this repo: `third_party/PS2X_lib/`
- Install location (macOS): `~/Documents/Arduino/libraries/PS2X_lib/`
- Expected header path after install: `~/Documents/Arduino/libraries/PS2X_lib/PS2X_lib.h`
- Restart Arduino IDE after installing so the library index refreshes
- The vendored copy is kept in-repo for traceability, but Arduino IDE may still prefer the sketchbook-installed library unless your build tooling is configured to use the repo copy directly

### Build with Arduino CLI

This project has been verified to build from the terminal/VS Code using:

- `arduino-cli compile --fqbn arduino:avr:mega "/Users/micahbreitenstein/Downloads/to Micah/MEGA__master"`

This targets the Arduino Mega 2560 (`arduino:avr:mega`) and compiles the master sketch without needing to open Arduino IDE.

### Build with VS Code

- Install an Arduino extension in VS Code (workspace recommendations are in `.vscode/extensions.json`)
- Use the default build task in `.vscode/tasks.json`
- Run build with `Cmd+Shift+B` and choose `Arduino CLI: Build Mega master` (or let it run as the default build task)
- The task uses the same verified command above (`arduino-cli compile --fqbn arduino:avr:mega ...`)

## Controller Buttons (DualShock)

| Button | Function |
|---|---|
| D-pad left/right | Swing axis (solo, no pan) |
| D-pad left/right + SELECT held | Pan axis only (solo mode) |
| D-pad left/right (no SELECT) | Swing + pan combined |
| D-pad up/down | Lift axis (solo, no tilt) |
| D-pad up/down + SELECT held | (reserved for solo lift — see solo mode logic) |
| D-pad up/down (no SELECT) | Lift + tilt combined |
| Right stick X | Pan trim (during swing) or pan-only at extremes (left stick = pan left, right stick = pan right) |
| Right stick Y | Tilt trim (during lift) or tilt-only at extremes (stick up = tilt up, stick down = tilt down) |
| Triangle / Cross | Focus left / right |
| Square / Circle | Focus speed down / up |
| L1 / L2 | Pan + swing speed up / down |
| R1 / R2 | Lift + tilt speed up / down |
| **L1 + L2 + R1 + R2** | **Emergency stop: immediately stops all motors, cancels timelapse/bounce, and clears interval rumble. Normal controls resume after release.** |
| **START + SELECT + SQUARE** | **Toggle controller rumble mute/unmute. Serial logs stay enabled.** |
| SELECT release | Start timelapse mode (stick position selects mode 1–8) |
| START release | Start bounce/moco mode (stick position selects mode 1–8) |
| L3 (left stick click) | **Normal mode:** set bounce distance endpoint (ends stage 0, starts stage 1); double medium pulse confirms lock. **Drone Mode:** record current axis positions as a Flowlapse waypoint; short rumble confirms. |
| **R3 (press right joystick inward / right stick click)** | **Toggle Drone Mode ON/OFF. Enter = single medium pulse; Exit = double medium pulse. While Drone Mode is active, timelapse and bounce are locked out, and both joysticks control motion at multiple speed levels based on stick deflection.** |

### Timelapse Modes (SELECT release)

Stick position at moment of SELECT release selects the move:

| Stick position | Mode |
|---|---|
| Left + down | 1: swing left, boom down |
| Left + up | 2: swing left, boom up |
| Right + up | 3: swing right, boom up |
| Right + down | 4: swing right, boom down |
| Full left (X=0) | 5: swing left only |
| Full up (Y=0) | 6: boom up only |
| Full right (X=255) | 7: swing right only |
| Full down (Y=255) | 8: boom down only |

- **Stop/reset:** hold **L1 + L2 + R1 + R2** for emergency stop

### Bounce / MoCo Modes (START release)

Same stick positions as timelapse modes above, triggered with START instead of SELECT.
- **Stage 0:** rig moves in the initial direction; press L3 to mark the travel distance — a double medium pulse confirms the endpoint is locked
- **Stage 1:** rig bounces back and forth over the recorded distance automatically
- **Stop/reset:** hold **L1 + L2 + R1 + R2** for emergency stop
- **Minimum endpoint duration:** L3 endpoint capture requires at least `150 ms` of stage-0 travel; shorter taps are rejected with deny rumble + Serial warning

## New Features

### Emergency stop combo

Use this at any time for a hard stop:

- Hold **L1 + L2 + R1 + R2** together
- This immediately:
	- stops all motor outputs
	- cancels active timelapse mode (if running)
	- cancels active bounce mode (if running)
	- clears interval rumble feedback
- Recovery behavior:
	- while all 4 buttons are held, other inputs are ignored
	- after releasing the combo, manual controls respond again immediately
	- on combo release, Serial prints `EMERGENCY STOP RELEASED | controls re-enabled`
	- on combo release, a short confirmation rumble plays to confirm controls are re-enabled
	- timelapse/bounce stay canceled until you start them again (SELECT release / START release)

### Rumble mute toggle

Use this when you want quiet operation but still want Serial feedback:

- Hold **START + SELECT + SQUARE** together to toggle rumble mute
- When muted:
	- controller vibration output is disabled
	- Serial log messages still print normally
	- interval / `stepDist` / limit / deny / drone-mode / release rumble patterns are suppressed on the controller
- When unmuted:
	- controller rumble output resumes normally
	- a short confirmation rumble plays when rumble is turned back on

### Drone mode (R3 toggle)

Use this for dual-stick flying-drone style control.

- Press **R3** to toggle Drone Mode ON/OFF
- Enter feedback: **1 medium pulse**
- Exit feedback: **2 medium pulses**
- Enter behavior: active timelapse/bounce are reset and locked out while Drone Mode is active
- Both left and right sticks support multiple speed levels based on stick deflection (low / medium / high)
- Left stick in Drone Mode:
	- X controls swing direction + proportional speed
	- Y controls lift direction + proportional speed
- Right stick in Drone Mode:
	- X controls pan direction + proportional speed
	- Y controls tilt direction + proportional speed
- Focus stays on Triangle/Cross with Square/Circle speed control
- In Drone Mode, **SELECT/START are remapped to Flowlapse controls** (timelapse/bounce mode selection is ignored while Drone Mode is active)
- Drone speed modifiers while in Drone Mode:
	- Hold `L2` for **micro-motion mode**: ultra-smooth cinematic movement at 0.25x speed (macro shots, interview push-ins, product framing)
	- Hold `R2` for temporary boost mode (one speed tier faster, never above per-axis cap)
	- If both `L2` and `R2` are held together, L2 takes priority (micro-motion)
- Exit Drone Mode by pressing **R3** again (toggle off)

### Flowlapse (Drone Mode only)

Flowlapse is a waypoint timelapse system for recording a multi-axis camera path and replaying it as a timelapse. It runs using the four Drone Mode axes (swing/lift/pan/tilt).

- Waypoint recording (up to 8 points):
	- Press **L3** to record the current location estimate as a waypoint
	- Each waypoint record gives a short rumble confirmation; axis positions (swing/lift/pan/tilt) print to serial
	- Waypoints are **session-only** (cleared on power cycle / drone mode restart)
- Control flow:
	- **1st SELECT**: stop waypoint recording (requires at least 2 waypoints); controller rumbles once per recorded waypoint as a count confirmation
	- **2nd SELECT**: run preview pass through recorded waypoints for visual check
	- **START while preview is running**: force-complete preview and move to **READY_FOR_CAPTURE** (useful if preview appears stuck)
	- **START during capture**: pause capture; motors stop, timer halts. Press START again to resume
	- **START + SELECT + TRIANGLE**: toggle frame-count mode at runtime (no recompile needed); this changes whether preview/capture use equal-distance frame stops or normal waypoint stepping
	- While in ready states (after recording stop and after preview), tap **D-pad RIGHT/LEFT** to jog one waypoint forward/backward (`current/total` waypoint index prints to Serial)
	- **SELECT + D-pad UP/DOWN** (Drone Mode) increases/decreases Flowlapse dwell by 250 ms per press (0 to 5000 ms)
	- Dwell adjustment rumble encodes current dwell in 250 ms steps (e.g., `1000 ms` = 4 pulses); `0 ms` plays a double pulse (disabled)
	- **START**: run actual Flowlapse capture when in **READY_FOR_CAPTURE** (if you skipped preview with START, press START again to begin capture)
	- **L1 + R1**: wipe the full Flowlapse course and re-arm recording
	- **L2 + R2 (1st press)**: move the rig back to the **last** recorded waypoint
	- **L2 + R2 (2nd press)**: rumble + delete that last point (repeat this two-step cycle to walk the path backward)
	- **START + R1**: return to the first waypoint (jog from current position back to waypoint 1); only active when not already running preview/capture/undo/jog
	- **L2 hold during preview**: speed up preview playback to 3x normal speed for quick testing and path verification (release L2 to resume normal speed)
- Capture pause/resume (useful if wind changes, subject moves, or lighting shifts):
	- Press **START** during active capture to **pause**: motors stop immediately, capture timer halts
	- Press **START** again to **resume** from the paused position (no loss of already-captured frames)
	- Hold **L3** for `FLOWLAPSE_L3_RESET_HOLD_MS` to quick-reset: clear course + re-arm recording
- Capture behavior:
	- Trigger/pause uses `timelapseIntervalSeconds` (same camera interval timing model)
	- Move slice uses `stepDist` (same move-duration concept as normal timelapse)
	- Segment motion uses smooth ease-in/ease-out timing (slow start/end, faster mid-segment) when `FLOWLAPSE_EASE_IN_OUT_ENABLED = true`
	- `FLOWLAPSE_EASE_STRENGTH` blends easing intensity (`0.0` ≈ linear, `1.0` = strong easing)
	- With `FLOWLAPSE_ARC_LENGTH_SAMPLING_ENABLED = true`, curved capture uses arc-length-based sampling for more path-even spatial progression
	- With `FLOWLAPSE_FRAME_COUNT_MODE_ENABLED = true`, capture places stops at equal path distances and takes exactly `FLOWLAPSE_FRAME_COUNT_TARGET` photos (including first and last stop)
	- The compile-time `FLOWLAPSE_FRAME_COUNT_MODE_ENABLED` value is now just the boot default; you can toggle the live mode from the controller with **START + SELECT + TRIANGLE**
	- Preview also follows the actual evenly spaced frame-count stops when frame-count mode is enabled, rather than only previewing recorded waypoints
	- On capture start in frame-count mode, Serial logs the computed plan, e.g. `Frame-count: 48 frames | spacing=12.4 units`
	- Frame-count completion feedback: Serial prints `Frame-count capture complete (N frames) — mode exited` and plays a distinct long+short rumble pulse
	- While frame-count mode is active, loop modes are explicitly disabled for the run (`FLOWLAPSE_LOOP_CAPTURE` and `FLOWLAPSE_PING_PONG_LOOP` are ignored with Serial notice)
	- Motion is interpolated per-axis between recorded waypoints over repeated frame cycles
	- With `FLOWLAPSE_CURVED_PATH_ENABLED = true`, capture move phase follows a smooth Catmull-Rom curved path through waypoints (fallback to linear path when fewer than 3 waypoints)
	- Optional dwell/settle pause before each trigger uses `flowlapseDwellMs` (controller-adjustable in Drone Mode)
	- Serial capture progress logs include an estimated remaining time (`eta=Ns`)
	- Set `FLOWLAPSE_LOOP_CAPTURE = true` to auto-restart capture from waypoint 0 after each full pass (default `false`)
	- Set `FLOWLAPSE_PING_PONG_LOOP = true` to continuously reverse direction at path ends (forward/backward loop)
	- Set `FLOWLAPSE_RETURN_TO_FIRST_WAYPOINT_ON_COMPLETE = true` to auto-return to waypoint 1 after non-loop capture completes
- While waiting in READY_FOR_PREVIEW or READY_FOR_CAPTURE, focus axis (Triangle/Cross/Square/Circle) stays fully responsive
- Safety behavior:
	- Playback speed is capped conservatively (no aggressive instant max-speed jumps)
	- Speed tiers ramp gradually, including direction reversals (e.g., opposite endpoints)
	- Emergency stop (**L1 + L2 + R1 + R2**) immediately cancels Flowlapse motion

#### Drone mode direction reference

Use this quick map while flying manually (assuming DIP reverse switches are OFF):

| Stick movement | Axis result |
|---|---|
| Left stick LEFT | Swing left |
| Left stick RIGHT | Swing right |
| Left stick UP | Lift up |
| Left stick DOWN | Lift down |
| Right stick LEFT | Pan left |
| Right stick RIGHT | Pan right |
| Right stick UP | Tilt up |
| Right stick DOWN | Tilt down |

Speed level is based on how far you push either stick from center (small deflection = slower, large deflection = faster).

#### Drone mode tuning constants (firmware)

These constants live in [MEGA__master/MEGA__master.ino](MEGA__master/MEGA__master.ino) and control Drone Mode feel:

- Per-axis expo curves:
	- `DRONE_SWING_EXPO_PERCENT` (currently `65`)
	- `DRONE_LIFT_EXPO_PERCENT` (currently `65`)
	- `DRONE_PAN_EXPO_PERCENT` (currently `65`)
	- `DRONE_TILT_EXPO_PERCENT` (currently `65`)
	- Higher = softer response near center, more ramp near edge
	- Lower = more linear response
- Per-axis deadbands:
	- `DRONE_SWING_DEADBAND` (currently `12`)
	- `DRONE_LIFT_DEADBAND` (currently `14`)
	- `DRONE_PAN_DEADBAND` (currently `10`)
	- `DRONE_TILT_DEADBAND` (currently `10`)
	- Inside deadband, both direction and speed are forced to stop for that axis
- Per-axis max speed caps:
	- `DRONE_SWING_MAX_SPEED_TIER` (currently `DRONE_SPEED_TIER_MED`)
	- `DRONE_LIFT_MAX_SPEED_TIER` (currently `DRONE_SPEED_TIER_MED`)
	- `DRONE_PAN_MAX_SPEED_TIER` (currently `DRONE_SPEED_TIER_HIGH`)
	- `DRONE_TILT_MAX_SPEED_TIER` (currently `DRONE_SPEED_TIER_MED`)
	- Available tiers: `DRONE_SPEED_TIER_STOP`, `DRONE_SPEED_TIER_MED`, `DRONE_SPEED_TIER_HIGH`
- Speed tier thresholds (expo-space):
	- `DRONE_SPEED_TIER_MED_THRESHOLD` (currently `43`)
	- `DRONE_SPEED_TIER_HIGH_THRESHOLD` (currently `86`)
- Modifier toggles:
	- `DRONE_ENABLE_PRECISION_MODIFIER` (currently `true`)
	- `DRONE_ENABLE_BOOST_MODIFIER` (currently `true`)
	- `DRONE_L2_R2_NEUTRAL_MODE` (currently `true`)
- Logging:
	- `DRONE_IDLE_TIMEOUT_MS` (disabled; idle auto-exit removed)
	- `DRONE_SERIAL_LOG_ENABLED` (currently `true`) — set `false` to silence runtime drone logs (axis movement and modifier state). Boot tuning profile always prints regardless.
	- `DRONE_MICRO_MOTION_SPEED_RATIO` (currently `0.25`) — L2 micro-motion multiplier for ultra-smooth cinematic control
- Flowlapse safety constants:
	- `FLOWLAPSE_MAX_WAYPOINTS` (currently `8`)
	- `FLOWLAPSE_LOOP_CAPTURE` (currently `false`) — set `true` to loop capture continuously from start after each pass
	- `FLOWLAPSE_WAYPOINT_RUMBLE_ON_MS` (currently `90`) — waypoint-confirm rumble on-time
	- `FLOWLAPSE_WAYPOINT_RUMBLE_TOTAL_MS` (currently `180`) — waypoint-confirm rumble total pulse window
	- `FLOWLAPSE_WAYPOINT_RUMBLE_PULSES` (currently `1`) — waypoint-confirm pulse count
	- `FLOWLAPSE_PREVIEW_POINT_HOLD_MS` (currently `700`) — hold duration at each preview waypoint
	- `FLOWLAPSE_MAX_SPEED_TIER` (currently `DRONE_SPEED_TIER_MED`)
	- `FLOWLAPSE_TIER_RAMP_INTERVAL_MS` (currently `450`)
	- `FLOWLAPSE_AXIS_STOP_TOLERANCE` (currently `1.0`) — target-reached tolerance band
	- `FLOWLAPSE_AXIS_MED_ERROR` / `FLOWLAPSE_AXIS_HIGH_ERROR` (currently `4.0` / `12.0`) — error bands for tier selection
	- `FLOWLAPSE_MIN_WAYPOINT_SEPARATION` (currently `2.5`) — minimum spacing between recorded waypoints
	- `FLOWLAPSE_MANUAL_TRACK_RATE_UNITS_PER_SEC` / `FLOWLAPSE_MED_RATE_UNITS_PER_SEC` / `FLOWLAPSE_HIGH_RATE_UNITS_PER_SEC` (currently `14` / `10` / `18`) — open-loop motion model rates
	- `FLOWLAPSE_CAPTURE_PROGRESS_LOG_MS` (currently `2000`) — progress/ETA serial print cadence
	- `FLOWLAPSE_CURVED_PATH_ENABLED` (currently `true`) — enables smooth curved interpolation during capture move phase
	- `FLOWLAPSE_WAYPOINT_DWELL_MS` (currently `0`) — boot default dwell in ms before each trigger
	- `FLOWLAPSE_DWELL_ADJUST_INCREMENT_MS` (currently `250`) — controller dwell step size
	- `FLOWLAPSE_DWELL_MAX_MS` (currently `5000`) — controller dwell upper cap
	- `FLOWLAPSE_PREVIEW_SPEED_SCALE` (currently `0.70`) — preview-only motion speed multiplier
	- `FLOWLAPSE_L3_RESET_HOLD_MS` (currently `1500`) — L3 hold duration for quick reset/re-arm
	- `FLOWLAPSE_EASE_IN_OUT_ENABLED` (currently `true`) — applies smooth ease-in/ease-out timing to each capture segment
	- `FLOWLAPSE_EASE_STRENGTH` (currently `1.0`) — easing blend (`0.0` linear to `1.0` strong easing)
	- `FLOWLAPSE_PING_PONG_LOOP` (currently `false`) — reverses direction automatically at each path edge
	- `FLOWLAPSE_RETURN_TO_FIRST_WAYPOINT_ON_COMPLETE` (currently `false`) — auto-return to waypoint 1 after a non-loop run
	- `FLOWLAPSE_ARC_LENGTH_SAMPLING_ENABLED` (currently `true`) — enables arc-length reparameterization in curved capture segments
	- `FLOWLAPSE_ARC_LENGTH_QUALITY_PRESET` (currently `FLOWLAPSE_ARC_LENGTH_QUALITY_MED`) — simple quality selector (`LOW`, `MED`, `HIGH`)
	- `FLOWLAPSE_ARC_LENGTH_TABLE_STEPS` (currently `16`) — auto-derived from preset (`LOW=8`, `MED=16`, `HIGH=24`)
	- Preset intent: **Low = faster/lighter**, **Med = balanced**, **High = smoothest/heaviest**
	- `FLOWLAPSE_FRAME_COUNT_MODE_ENABLED` (currently `false`) — when enabled, capture progression uses equal-distance frame stops
	- `FLOWLAPSE_FRAME_COUNT_TARGET` (currently `48`) — total number of capture stops/frames in frame-count mode
	- `FLOWLAPSE_FRAMECOUNT_AUTO_EXIT` (currently `true`) — exits frame-count mode after completion (set `false` to remain armed for rerun)

Quick tuning guide:

- If an axis drifts at center, increase that axis deadband by `1-2`
- If stick response feels too twitchy near center, increase that axis `DRONE_*_EXPO_PERCENT`
- If controls feel sluggish, decrease that axis `DRONE_*_EXPO_PERCENT` or reduce deadband on that axis
- If an axis is too aggressive at full stick, lower that axis `DRONE_*_MAX_SPEED_TIER`
- On boot, Serial prints expo/deadband/max-tier/modifier/threshold/idle-timeout/log-flag summaries so you can confirm the active profile
- On boot, Serial prints a compact Flowlapse mode summary line, e.g. `Flowlapse summary | ease=Y strength=1.00 loop=N frame=N fexit=Y arc=Y ping-pong=N return-home=N`
	- Field mapping: `ease` → `FLOWLAPSE_EASE_IN_OUT_ENABLED`, `strength` → `FLOWLAPSE_EASE_STRENGTH`, `loop` → `FLOWLAPSE_LOOP_CAPTURE`, `frame` → `FLOWLAPSE_FRAME_COUNT_MODE_ENABLED`, `fexit` → `FLOWLAPSE_FRAMECOUNT_AUTO_EXIT`, `arc` → `FLOWLAPSE_ARC_LENGTH_SAMPLING_ENABLED`, `ping-pong` → `FLOWLAPSE_PING_PONG_LOOP`, `return-home` → `FLOWLAPSE_RETURN_TO_FIRST_WAYPOINT_ON_COMPLETE`
- On boot, Serial also prints DIP axis reversal state for all 5 axes: `Boot axis reversal | swing=N pan=N lift=N tilt=N focus=N` (`Y` = reversed, `N` = normal)
- When `DRONE_SERIAL_LOG_ENABLED = true`, the Mega prints edge-triggered events: per-axis start/stop movement and L2/R2 modifier state changes

#### Starter profiles (copy these values)

Pick one profile and set the constants in [MEGA__master/MEGA__master.ino](MEGA__master/MEGA__master.ino):

- **Safe / Indoor (slow + smooth):**
	- `DRONE_SWING_EXPO_PERCENT = 75`
	- `DRONE_LIFT_EXPO_PERCENT = 75`
	- `DRONE_PAN_EXPO_PERCENT = 75`
	- `DRONE_TILT_EXPO_PERCENT = 75`
	- `DRONE_SWING_DEADBAND = 14`
	- `DRONE_LIFT_DEADBAND = 16`
	- `DRONE_PAN_DEADBAND = 12`
	- `DRONE_TILT_DEADBAND = 12`
	- `DRONE_SWING_MAX_SPEED_TIER = DRONE_SPEED_TIER_MED`
	- `DRONE_LIFT_MAX_SPEED_TIER = DRONE_SPEED_TIER_MED`
	- `DRONE_PAN_MAX_SPEED_TIER = DRONE_SPEED_TIER_MED`
	- `DRONE_TILT_MAX_SPEED_TIER = DRONE_SPEED_TIER_MED`

- **Balanced (default feel):**
	- `DRONE_SWING_EXPO_PERCENT = 65`
	- `DRONE_LIFT_EXPO_PERCENT = 65`
	- `DRONE_PAN_EXPO_PERCENT = 65`
	- `DRONE_TILT_EXPO_PERCENT = 65`
	- `DRONE_SWING_DEADBAND = 12`
	- `DRONE_LIFT_DEADBAND = 14`
	- `DRONE_PAN_DEADBAND = 10`
	- `DRONE_TILT_DEADBAND = 10`
	- `DRONE_SWING_MAX_SPEED_TIER = DRONE_SPEED_TIER_MED`
	- `DRONE_LIFT_MAX_SPEED_TIER = DRONE_SPEED_TIER_MED`
	- `DRONE_PAN_MAX_SPEED_TIER = DRONE_SPEED_TIER_HIGH`
	- `DRONE_TILT_MAX_SPEED_TIER = DRONE_SPEED_TIER_MED`

- **Fast / Outdoor (more aggressive):**
	- `DRONE_SWING_EXPO_PERCENT = 50`
	- `DRONE_LIFT_EXPO_PERCENT = 50`
	- `DRONE_PAN_EXPO_PERCENT = 50`
	- `DRONE_TILT_EXPO_PERCENT = 50`
	- `DRONE_SWING_DEADBAND = 10`
	- `DRONE_LIFT_DEADBAND = 12`
	- `DRONE_PAN_DEADBAND = 8`
	- `DRONE_TILT_DEADBAND = 8`
	- `DRONE_SWING_MAX_SPEED_TIER = DRONE_SPEED_TIER_HIGH`
	- `DRONE_LIFT_MAX_SPEED_TIER = DRONE_SPEED_TIER_MED`
	- `DRONE_PAN_MAX_SPEED_TIER = DRONE_SPEED_TIER_HIGH`
	- `DRONE_TILT_MAX_SPEED_TIER = DRONE_SPEED_TIER_HIGH`

### Controller-adjustable timelapse interval (no reflash needed)

You can now change the timelapse interval directly from the controller while no auto mode is active.

- **Increase interval:** hold **START** and tap **D-pad UP**
- **Decrease interval:** hold **START** and tap **D-pad DOWN**
- **Allowed range:** 1 to 99 seconds
- **Safety rule:** this adjustment is only active when both timelapse and bounce are idle
- **Persistence:** saved on change and restored at boot
- **Limit feedback:** trying to go below/above range gives a distinct double-short rumble
- **Lockout feedback:** trying this combo while timelapse or bounce is active gives a distinct triple-short deny rumble
- **R3 behavior:** toggles Drone Mode ON/OFF (does not cancel timelapse/bounce directly)

When changed, the Mega prints the value over Serial as:

- `Timelapse interval (seconds) = X`

### Rumble interval feedback (time-code)

After each interval change, the controller rumbles in a pattern that encodes the selected seconds:

- **Long rumble = 10 seconds**
- **Short rumble sequence = 1-second units**

Examples:

- **15 seconds** → 1 long + 5 short
- **20 seconds** → 2 long
- **24 seconds** → 2 long + 4 short

Interpretation guide:

- Count the number of long rumbles first (tens place)
- Count the number of short rumbles next (ones place)
- Total seconds = `(long count × 10) + short count`
- If you switch from interval adjustment to `stepDist` adjustment (or back), a brief silent separator pause is inserted before the new rumble pattern starts

### Settings replay

Replay the current interval and `stepDist` rumble patterns on demand (read-only, no values change):

- Hold **L1 + L2 + CIRCLE** together to replay
- This triggers:
	- The interval rumble pattern (current `timelapseIntervalSeconds` encoded as long/short rumbles)
	- Followed immediately by the `stepDist` rumble pattern (current `stepDist` encoded as long pulses)
	- Serial prints: `Settings replay: interval=Xs, stepDist=Yms` (e.g., `Settings replay: interval=15s, stepDist=50ms`)
- **Use case:** verify your current settings without adjusting them; useful when switching between different rigs or recalls
- **Safety:** this is purely informational; no motion or configuration changes occur
- **Idle-only lockout:** replay is blocked while timelapse or bounce is active; a triple-short deny rumble + Serial message confirms lockout

### Controller-adjustable timelapse move time (`stepDist`) (no reflash needed)

You can also change the timelapse move-active duration (`stepDist`) directly from the controller.

- **Increase `stepDist`:** hold **SELECT** and tap **D-pad RIGHT**
- **Decrease `stepDist`:** hold **SELECT** and tap **D-pad LEFT**
- **Step size:** **10 ms** per press
- **Allowed range:** 20 ms to 150 ms
- **Safety rule:** this adjustment is only active when both timelapse and bounce are idle
- **Persistence:** saved on change and restored at boot
- **Limit feedback:** trying to go below/above range gives a distinct double-short rumble
- **Lockout feedback:** trying this combo while timelapse or bounce is active gives a distinct triple-short deny rumble
- **Rumble feedback:** long-pulse count encodes `stepDist` in 10 ms units (`stepDist / 10`)
	- `20 ms` → 2 long pulses
	- `100 ms` → 10 long pulses
	- `150 ms` → 15 long pulses

When changed, the Mega prints the value over Serial as:

- `Timelapse stepDist (ms) = X`

### Rumble mute persistence

Rumble mute now persists across power cycles.

- Toggle rumble mute as usual in the runtime controls
- **Persistence:** the mute preference is saved immediately when toggled
- On boot, the saved preference is restored automatically

### Reset persisted defaults (controller combo)

You can reset persisted settings to defaults without reflashing.

- Hold **START + SELECT + CIRCLE** together for **1.5 seconds** while auto modes are idle
- Resets to:
	- `timelapseIntervalSeconds = 15`
	- `stepDist = 50`
	- `rumbleMuted = false`
- The reset values are written to EEPROM immediately
- **Lockout:** while timelapse, bounce, or drone mode is active, reset is denied and values are unchanged

## Project Details Sheet

- Full project details are documented here: https://docs.google.com/spreadsheets/d/1BU9yWQd8groFjTa8OWagQ6Nst10-R33pP6oTYL_nhLQ/edit?usp=sharing

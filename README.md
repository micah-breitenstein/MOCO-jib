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
| L3 (left stick click) | Set bounce distance endpoint (ends stage 0, starts stage 1). A double medium pulse confirms the endpoint was locked. |
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
	- Hold `L2` for temporary precision mode (one speed tier slower)
	- Hold `R2` for temporary boost mode (one speed tier faster, never above per-axis cap)
	- If both `L2` and `R2` are held together, they resolve to neutral (no modifier)
- Exit Drone Mode by pressing **R3** again (toggle off)

#### Flowlapse (Drone Mode only)

Flowlapse is a waypoint timelapse path run using the four Drone axes (swing/lift/pan/tilt).

- Waypoint recording (up to 8 points):
	- Press **L3** to record the current location estimate as a waypoint
	- Each waypoint record gives a short rumble confirmation
	- Waypoints are **session-only** (cleared on power cycle / drone mode restart)
- Control flow:
	- **1st SELECT**: stop waypoint recording (requires at least 2 waypoints)
	- **2nd SELECT**: run preview pass through recorded waypoints for visual check
	- **START**: run actual Flowlapse capture after preview completes
	- **L1 + R1**: wipe the full Flowlapse course and re-arm recording
	- **L2 + R2 (1st press)**: move the rig back to the **last** recorded waypoint
	- **L2 + R2 (2nd press)**: rumble + delete that last point (repeat this two-step cycle to walk the path backward)
- Capture behavior:
	- Trigger/pause uses `timelapseIntervalSeconds` (same camera interval timing model)
	- Move slice uses `stepDist` (same move-duration concept as normal timelapse)
	- Motion is interpolated per-axis between recorded waypoints over repeated frame cycles
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
- Flowlapse safety constants:
	- `FLOWLAPSE_MAX_WAYPOINTS` (currently `8`)
	- `FLOWLAPSE_MAX_SPEED_TIER` (currently `DRONE_SPEED_TIER_MED`)
	- `FLOWLAPSE_TIER_RAMP_INTERVAL_MS` (currently `450`)
	- `FLOWLAPSE_AXIS_MED_ERROR` and `FLOWLAPSE_AXIS_HIGH_ERROR` (error bands for tier selection)
	- `FLOWLAPSE_MANUAL_TRACK_RATE_UNITS_PER_SEC` / `FLOWLAPSE_MED_RATE_UNITS_PER_SEC` / `FLOWLAPSE_HIGH_RATE_UNITS_PER_SEC` (currently `14` / `10` / `18`, open-loop motion model rates)

Quick tuning guide:

- If an axis drifts at center, increase that axis deadband by `1-2`
- If stick response feels too twitchy near center, increase that axis `DRONE_*_EXPO_PERCENT`
- If controls feel sluggish, decrease that axis `DRONE_*_EXPO_PERCENT` or reduce deadband on that axis
- If an axis is too aggressive at full stick, lower that axis `DRONE_*_MAX_SPEED_TIER`
- On boot, Serial prints expo/deadband/max-tier/modifier/threshold/idle-timeout/log-flag summaries so you can confirm the active profile
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
- **Limit feedback:** trying to go below/above range gives a distinct double-short rumble
- **Lockout feedback:** trying this combo while timelapse or bounce is active gives a distinct triple-short deny rumble
- **Rumble feedback:** long-pulse count encodes `stepDist` in 10 ms units (`stepDist / 10`)
	- `20 ms` → 2 long pulses
	- `100 ms` → 10 long pulses
	- `150 ms` → 15 long pulses

When changed, the Mega prints the value over Serial as:

- `Timelapse stepDist (ms) = X`

## Project Details Sheet

- Full project details are documented here: https://docs.google.com/spreadsheets/d/1BU9yWQd8groFjTa8OWagQ6Nst10-R33pP6oTYL_nhLQ/edit?usp=sharing

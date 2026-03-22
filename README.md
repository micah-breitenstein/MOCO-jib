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

## Controller Buttons (DualShock)

| Button | Function |
|---|---|
| D-pad left/right | Swing axis (solo, no pan) |
| D-pad left/right + SELECT held | Pan axis only (solo mode) |
| D-pad left/right (no SELECT) | Swing + pan combined |
| D-pad up/down | Lift axis (solo, no tilt) |
| D-pad up/down + SELECT held | (reserved for solo lift — see solo mode logic) |
| D-pad up/down (no SELECT) | Lift + tilt combined |
| Right stick X | Pan trim (during swing) or pan-only at extremes |
| Right stick Y | Tilt trim (during lift) or tilt-only at extremes |
| Triangle / Cross | Focus left / right |
| Square / Circle | Focus speed down / up |
| L1 / L2 | Pan + swing speed up / down |
| R1 / R2 | Lift + tilt speed up / down |
| **L1 + L2 + R1 + R2** | **Emergency stop: immediately stops all motors, cancels timelapse/bounce, and clears interval rumble** |
| SELECT release | Start timelapse mode (stick position selects mode 1–8) |
| START release | Start bounce/moco mode (stick position selects mode 1–8) |
| L3 (left stick click) | Set bounce distance endpoint (ends stage 0, starts stage 1) |
| **R3 (right stick click)** | **Cancel active timelapse/bounce (stops motors + resets). If no auto mode is active, it cancels interval rumble feedback.** |

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

- **Cancel:** press R3 at any time to stop timelapse and reset

### Bounce / MoCo Modes (START release)

Same stick positions as timelapse modes above, triggered with START instead of SELECT.
- **Stage 0:** rig moves in the initial direction; press L3 to mark the travel distance
- **Stage 1:** rig bounces back and forth over the recorded distance automatically
- **Cancel:** press R3 at any time to stop bounce and reset

## New Features

### Emergency stop combo

Use this at any time for a hard stop:

- Hold **L1 + L2 + R1 + R2** together
- This immediately:
	- stops all motor outputs
	- cancels active timelapse mode (if running)
	- cancels active bounce mode (if running)
	- clears interval rumble feedback

### Controller-adjustable timelapse interval (no reflash needed)

You can now change the timelapse interval directly from the controller while no auto mode is active.

- **Increase interval:** hold **START** and tap **D-pad UP**
- **Decrease interval:** hold **START** and tap **D-pad DOWN**
- **Allowed range:** 1 to 99 seconds
- **Safety rule:** this adjustment is only active when both timelapse and bounce are idle
- **Limit feedback:** trying to go below/above range gives a distinct double-short rumble
- **R3 behavior priority:**
	- If timelapse is active, R3 cancels timelapse
	- Else if bounce is active, R3 cancels bounce
	- Else (idle), R3 cancels interval rumble feedback

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

### Controller-adjustable timelapse move time (`stepDist`) (no reflash needed)

You can also change the timelapse move-active duration (`stepDist`) directly from the controller.

- **Increase `stepDist`:** hold **SELECT** and tap **D-pad RIGHT**
- **Decrease `stepDist`:** hold **SELECT** and tap **D-pad LEFT**
- **Step size:** **10 ms** per press
- **Allowed range:** 20 ms to 150 ms
- **Safety rule:** this adjustment is only active when both timelapse and bounce are idle
- **Limit feedback:** trying to go below/above range gives a distinct double-short rumble
- **Rumble feedback:** long-pulse count encodes `stepDist` in 10 ms units (`stepDist / 10`)
	- `20 ms` → 2 long pulses
	- `100 ms` → 10 long pulses
	- `150 ms` → 15 long pulses

When changed, the Mega prints the value over Serial as:

- `Timelapse stepDist (ms) = X`

## Project Details Sheet

- Full project details are documented here: https://docs.google.com/spreadsheets/d/1BU9yWQd8groFjTa8OWagQ6Nst10-R33pP6oTYL_nhLQ/edit?usp=sharing

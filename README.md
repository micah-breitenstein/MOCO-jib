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
| SELECT release | Start timelapse mode (stick position selects mode 1–8) |
| START release | Start bounce/moco mode (stick position selects mode 1–8) |
| L3 (left stick click) | Set bounce distance endpoint (ends stage 0, starts stage 1) |
| **R3 (right stick click)** | **Cancel active bounce — stops all motors and resets state** |

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

### Bounce / MoCo Modes (START release)

Same stick positions as timelapse modes above, triggered with START instead of SELECT.
- **Stage 0:** rig moves in the initial direction; press L3 to mark the travel distance
- **Stage 1:** rig bounces back and forth over the recorded distance automatically
- **Cancel:** press R3 at any time to stop bounce and reset

## Project Details Sheet

- Full project details are documented here: https://docs.google.com/spreadsheets/d/1BU9yWQd8groFjTa8OWagQ6Nst10-R33pP6oTYL_nhLQ/edit?usp=sharing

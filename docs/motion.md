# Motion

All motion uses the joint map (board/channel/dir from `wiggle`, stored in flash). Offsets follow each
joint's `dir`: **+ = lift up, knee up, swing forward**. Moves are linear ramps updated every 20 ms (one
servo frame); all joints in a step move together. **Any key aborts a move and holds position.**

## Stand up - `stand` / `stand step`

Also a power-harness test: each step moves one joint type on all 8 legs at once.

| Step | Joints | Target (from 1500 us) | Ramp |
|---|---|---|---|
| 1 | all 24 | 1500 (centre) | 500 ms |
| 2 | 8x Y (lift) | `STAND_LIFT_US` = +100 (up) | 500 ms |
| 3 | 8x K (knee) | `STAND_TUCK_US` = -300 (toward body; + moves them outward) | 500 ms |
| 4 | 8x Y (lift) | `STAND_PUSH_US` = -100 (down - lifts the body) | 750 ms |

500 ms pause between steps. `stand step` waits for Enter before each step (`q` stops) - use it for the
first runs to confirm each step moves the right way.

Knee "toward the body" is the knee-DOWN direction in the dir convention (verified 2026-10-03).

## Sit - `sit`

Y to centre (lowers the body, 750 ms), then K to centre, then X to centre (500 ms each).

## On boot

`BOOT_PULSE_US` (1500) on all 32 outputs, then if `BOOT_STAND` is true: 3 s countdown (any key cancels)
and the stand sequence. `BOOT_STAND` is **false** until the sequence has been verified with `stand step`.

All values: `include/config.h`.

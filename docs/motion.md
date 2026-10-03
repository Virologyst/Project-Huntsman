# Motion

All motion uses the joint map (board/channel/dir from `wiggle`, stored in flash). Every offset is from
the joint's **neutral** (its centre / trim) and follows its `dir`: **+ = lift up, knee up, swing forward**.
Neutral is 1500 for every joint except **FML X = 1650** (150 us forward) and **FMR X = 1400** (100 us
forward - FMR's dir is -1). The trims keep the front-middle legs clear of the back-middle legs when walking. Trim any joint with
`setneutral <leg> <joint> <us>` then `save`. Moves are linear ramps updated every 20 ms (one
servo frame); all joints in a step move together. **Any key aborts a move and holds position.**

## Stand up - `stand` / `stand step`

Also a power-harness test: each step moves one joint type on all 8 legs at once.

| Step | Joints | Target (offset from neutral) | Ramp |
|---|---|---|---|
| 1 | all 24 | neutral | 500 ms |
| 2 | 8x Y (lift) | `STAND_LIFT_US` = +200 (up) | 500 ms |
| 3 | 8x K (knee) | `STAND_TUCK_US` = -300 (toward body; + moves them outward) | 500 ms |
| 4 | 8x Y (lift) | `STAND_PUSH_US` = -100 (down - lifts the body) | 750 ms |

500 ms pause between steps. `stand step` waits for Enter before each step (`q` stops) - use it for the
first runs to confirm each step moves the right way.

Knee "toward the body" is the knee-DOWN direction in the dir convention (verified 2026-10-03).

## Sit - `sit`

Y to centre (lowers the body, 750 ms), then K to centre, then X to centre (500 ms each).

## Walk - `walk`, `back`, `turn left|right`

Joint-space **alternating tetrapod** from the stand pose (no inverse kinematics yet):

- Group A = **FL, BML, FMR, BR**; group B = **FML, BL, FR, BMR** - alternating along each side and across,
  so 4 legs are always on the ground.
- Half-cycle: swing group **lifts** (Y +`WALK_LIFT_US` above the stand pose, 200 ms), **swings** X to
  +`WALK_STRIDE_US` while the stance group **pushes** X to -`WALK_STRIDE_US` (400 ms), then **lowers**
  (200 ms). Then the groups swap.
- Direction comes from each X joint's dir (+ = forward). `back` reverses all; `turn left` swings left legs
  back and right legs forward; `turn right` the opposite.
- `walk 4` = 4 full cycles; `walk` alone = until a key. A key **finishes the current step** (never stops with
  legs in the air), then each group lifts and recentres X, ending in the stand pose.
- Requires the stand pose (Y and K exactly at stand values); otherwise it says to run `stand`.

| Constant | Value | Meaning |
|---|---|---|
| `WALK_LIFT_US` | 150 | Y up from the stand pose during swing |
| `WALK_STRIDE_US` | 120 | X each way from centre (~16 deg) |
| `WALK_LIFT_MS` | 200 | lift / lower ramp |
| `WALK_SWING_MS` | 400 | swing / push ramp |

Joint-space swing moves the feet in arcs about each hip (front/back legs also slide sideways a little).
Proper straight-line foot paths need IK and the leg segment lengths.

## On boot

`BOOT_PULSE_US` (1500) on all 32 outputs, then if `BOOT_STAND` is true: 3 s countdown (any key cancels)
and the stand sequence. `BOOT_STAND` is **true** (enabled 2026-10-03 after the tuned stand was verified).

All values: `include/config.h`.

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

Joint-space **alternating tetrapod** (no inverse kinematics yet):

- Group A = **FL, BML, FMR, BR**; group B = **FML, BL, FR, BMR** - alternating along each side and across,
  so 4 legs are always on the ground.
- **Walk pose.** A walk starts from the stand pose and moves each group (lift, place, lower - feet never
  drag) into a wider, lower walk pose; when it stops it returns the same way to the stand pose. Stand and
  sit are unchanged.
  - Middle legs (FML FMR BML BMR): knee out to `WALK_MID_KNEE_US` = -100 (stand is -300), feet reach
    out so each swing covers more ground.
  - Corner legs (FL FR BL BR): X base `WALK_CORNER_SPREAD_US` = 150 toward the head (front pair) / tail
    (back pair), knee `WALK_CORNER_KNEE_US` = -200 - they reach and pull at the front, push at the back.
  - The body sits lower in the walk pose (expected).
- Half-cycle: swing group **lifts** (Y +`WALK_LIFT_US` above the stand pose, 200 ms), **swings** X to
  base + stride while the stance group **pushes** X to base - stride (400 ms), then **lowers** (200 ms).
- Direction comes from each X joint's dir (+ = forward). `back` reverses all; `turn left` swings left legs
  back and right legs forward; `turn right` the opposite.
- `walk 4` = 4 full cycles; `walk` alone = until Enter. Stopping **finishes the current step**, then returns
  to the stand pose.
- Requires the stand pose (Y and K exactly at stand values); otherwise it says to run `stand`.

| Constant | Value | Meaning |
|---|---|---|
| `WALK_MID_KNEE_US` | -100 | middle-leg knee in the walk pose |
| `WALK_CORNER_KNEE_US` | -200 | corner-leg knee in the walk pose |
| `WALK_CORNER_SPREAD_US` | 150 | corner X base toward head / tail |
| `WALK_LIFT_US` | 150 | Y up from the stand pose during swing |
| `WALK_STRIDE_MID_US` | 150 | middle-leg X each way from base (~20 deg) |
| `WALK_STRIDE_CORNER_US` | 150 | corner-leg X each way from base |
| `WALK_LIFT_MS` | 200 | lift / lower ramp |
| `WALK_SWING_MS` | 400 | swing / push ramp |

Middle legs on each side are in opposite groups and swing toward each other every other step; the FML/FMR
X trims give clearance. The longer stride (150, was 120) reduces it - watch FML/BML and FMR/BMR.

Joint-space swing moves the feet in arcs about each hip. Straight-line foot paths need IK and the leg
segment lengths.

## On boot

`BOOT_PULSE_US` (1500) on all 32 outputs, then if `BOOT_STAND` is true: 3 s countdown (any key cancels)
and the stand sequence. `BOOT_STAND` is **true** (enabled 2026-10-03 after the tuned stand was verified).

All values: `include/config.h`.

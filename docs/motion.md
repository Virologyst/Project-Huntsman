# Motion

All motion uses the joint map (board/channel/dir from `wiggle`, stored in flash). Every offset is from
the joint's **neutral** (its centre / trim) and follows its `dir`: **+ = lift up, knee up, swing forward**.
Neutral is 1500 for every joint except **FML X = 1650** (150 us forward) and **FMR X = 1350** (150 us
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
  - Middle legs (FML FMR BML BMR): knee out to `WALK_MID_KNEE_US` = -50 (stand is -300; was -100), feet reach
    out so each swing covers more ground.
  - Front legs (FL FR): X base `WALK_FRONT_SPREAD_US` = 400 (~54 deg) toward the head, so the front feet
    come together ahead of the body before the first step and pull. Back legs (BL BR): X base
    `WALK_BACK_SPREAD_US` = 320 (~43 deg) toward the tail, they push. Knee `WALK_CORNER_KNEE_US` = -200 on all four.
  - Back-middle legs (BML BMR): X base `WALK_BACK_MID_SPREAD_US` = 60 toward the tail, to stay clear of
    the front-middle legs (FMR/BMR touched at stride 150).
  - The body sits lower in the walk pose (expected).
- Half-cycle: swing group **lifts** (Y +`WALK_LIFT_US` above the stand pose, 200 ms), **swings** X to
  base + stride while the stance group **pushes** X to base - stride (400 ms), then **lowers** (200 ms).
- **Front legs pull (`walk` / `back`).** FL and FR hold X at their base and do not stride on it. Swing:
  lift, knee out to `WALK_FRONT_REACH_KNEE_US`, lower to `WALK_FRONT_REACH_Y_US` - the foot is set down
  straight ahead. Stance: knee closes to `WALK_FRONT_PULL_KNEE_US` while Y moves to `WALK_FRONT_PULL_Y_US`,
  pulling the body to the foot. `back` runs the stroke the other way (tuck in the air, push out on the
  ground). Turns still swing the front legs on X. The K/Y values are **guesses** (no leg geometry yet):
  if the planted foot lifts during the pull make `WALK_FRONT_PULL_Y_US` more negative; if it jacks the
  front of the body up, less negative. A straight-line pull needs the femur and tibia lengths (IK).
- **Back legs push (`walk` / `back`).** BL and BR mirror the front pair: X holds at its base. Swing: lift,
  knee closes to `WALK_REAR_TUCK_KNEE_US`, lower to `WALK_REAR_TUCK_Y_US` - the foot is set down close in.
  Stance: knee opens to `WALK_REAR_PUSH_KNEE_US` while Y moves to `WALK_REAR_PUSH_Y_US`, pushing the body
  forward. `back` reverses the stroke; turns swing them on X. Values are **guesses** starting from the
  front pair's numbers - tune `WALK_REAR_PUSH_Y_US` / `WALK_REAR_TUCK_Y_US` separately (a push tends to
  lift the rear).
- Direction comes from each X joint's dir (+ = forward). `back` reverses all; `turn left` swings left legs
  back and right legs forward; `turn right` the opposite.
- `walk 4` = 4 full cycles; `walk` alone = until Enter. Stopping **finishes the current step**, then returns
  to the stand pose.
- Requires the stand pose (Y and K exactly at stand values); otherwise it says to run `stand`.

| Constant | Value | Meaning |
|---|---|---|
| `WALK_MID_KNEE_US` | -50 | middle-leg knee in the walk pose |
| `WALK_CORNER_KNEE_US` | -200 | corner-leg knee in the walk pose |
| `WALK_FRONT_SPREAD_US` | 400 | FL FR X base toward the head (~54 deg; was 220) |
| `WALK_FRONT_REACH_KNEE_US` | 200 | front-leg knee at full reach (was -50: knee travel doubled to 500 us) |
| `WALK_FRONT_REACH_Y_US` | -140 | front-leg Y when the reached foot is set down (was -100) |
| `WALK_FRONT_PULL_KNEE_US` | -550 | front-leg knee at the end of the pull (was -300: stroke +50%, inward end) |
| `WALK_FRONT_PULL_Y_US` | -20 | front-leg Y at the end of the pull (was -60) |
| `WALK_BACK_SPREAD_US` | 320 | BL BR X base toward the tail (~43 deg; 400 scraped the battery sides) |
| `WALK_REAR_TUCK_KNEE_US` | -550 | back-leg knee when set down, start of the push (was -300) |
| `WALK_REAR_TUCK_Y_US` | -20 | back-leg Y when the tucked foot is set down (was -60) |
| `WALK_REAR_PUSH_KNEE_US` | 200 | back-leg knee at the end of the push (was -50: knee travel doubled to 500 us) |
| `WALK_REAR_PUSH_Y_US` | -140 | back-leg Y at the end of the push (was -100) |
| `WALK_BACK_MID_SPREAD_US` | 60 | BML BMR X base toward the tail (middle-leg clearance) |
| `WALK_LIFT_US` | 350 | Y up from the stand pose during swing (was 150: only ~10 mm clearance) |
| `WALK_FRONT_LIFT_US` | 500 | same, for FL FR only (was 300) |
| `WALK_STRIDE_MID_US` | 150 | middle-leg X each way from base (~20 deg) |
| `WALK_FML_EXTRA_REACH_US` | 23 | FML only: forward end of its step 15% further (150 -> 173) |
| `WALK_STRIDE_CORNER_US` | 150 | corner-leg X each way from base (turns only) |
| `WALK_LIFT_MS` | 200 | lift / lower ramp |
| `WALK_SWING_MS` | 400 | swing / push ramp |

Middle legs on each side are in opposite groups and swing toward each other every other step; the FML/FMR
X trims (both 150 forward) plus `WALK_BACK_MID_SPREAD_US` give clearance: at full stride the pair is as
far apart as the left pair was at stride 120, which walked without contact. Untested - watch FML/BML and
FMR/BMR, and FL/FML and FR/FMR now the front legs sit further forward.

Joint-space swing moves the feet in arcs about each hip. Straight-line foot paths need IK and the leg
segment lengths.

Tuning 2026-10-04 (user, after floor tests): corners ~25 deg further toward head/tail, front/back K-Y
stroke 50% longer on the inward end (reach unchanged), FML reaches 15% further forward, all legs lift
much higher to step over obstacles. Back pair then eased to 320: at 400 BL/BR scraped the battery. Turns: corner X at base + stride can now reach the X limit (977 us)
and clamp there.

## Climb - `climb` (and the ToF sensor)

A forward-facing VL53L0X (docs/hardware.md) is sampled every 50 ms from `loop()` and between walk steps.
When the range drops below **`TOF_CLIMB_MM` = 300** it raises the **`CLIMB`** flag (`include/flags.h`, a
bit set; `tof` shows it). The flag is edge-triggered with hysteresis: it is raised once when something
enters the band and can't raise again until the range has gone past `TOF_CLEAR_MM` = 400; it drops if the
object goes away before anyone acted on it.

What happens on the flag (`TOF_AUTO_CLIMB` = true):

- A walk in progress stops the same way a key does - current step finishes, back to the stand pose.
- `loop()` then clears the flag and, if the robot is in the **stand pose**, runs `motion::climb()`.
  Sitting or mid-move it just prints `[tof] obstacle ... ignored` - stand first.
- `climb` on the console runs the same sequence by hand.

**The sequence itself is not written yet** - `motion::climb()` in `src/motion.cpp` is a stub that prints
and returns; the building blocks (`Pose`, `placeGroup`, `rampType`) and `cfg::CLIMB_*` constants are
listed in its TODO. It must start and end in the stand pose so walking can resume.

| Constant | Value | Meaning |
|---|---|---|
| `TOF_CLIMB_MM` | 300 | raise `CLIMB` below this |
| `TOF_CLEAR_MM` | 400 | re-arm above this |
| `TOF_MAX_MM` | 2000 | beyond = out of range |
| `TOF_PERIOD_MS` | 50 | sample interval |
| `TOF_AUTO_CLIMB` | true | `loop()` acts on the flag; false = flag only (`tof` / `climb` by hand) |
| `CLIMB_LIFT_US`, `CLIMB_RAMP_MS` | 300, 400 | placeholders for the sequence |

## Xbox controller

`walk` also takes an optional keep-going check: the controller (docs/controller.md) walks while the stick
or D-pad is held and stops the same way a key does - the current step finishes, then the legs go back to the stand pose.

## On boot

`BOOT_PULSE_US` (1500) on all 32 outputs, then if `BOOT_STAND` is true: 3 s countdown (any key cancels)
and the stand sequence. `BOOT_STAND` is **true** (enabled 2026-10-03 after the tuned stand was verified).

All values: `include/config.h`.

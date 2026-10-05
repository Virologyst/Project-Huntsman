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
- **Middle legs step at their own rate** (`WALK_MID_CYCLES` = 2, 2026-10-05): they take 2 full step cycles
  per corner half-cycle, with half the stride (`WALK_STRIDE_MID_US` = 75) so the body speed matches, and
  their feet brought in (`WALK_MID_KNEE_US` = -150). The pairs **BML+FMR** and **FML+BMR** alternate every
  middle step through the whole walk. Each middle step: lift + swing X (first half), lower (second half);
  the other pair pushes X back for the whole step. `WALK_MID_CYCLES` = 1 puts them back in step with the
  corners. **Balance caveat:** at 2x there is a moment each step when a front corner and the middle leg on
  the same side are both up (e.g. FL + FML, with BR + BMR), leaving FR / FMR / BML / BL - the FMR-BML line
  runs close to the centre. Watch for the front-left / back-right dipping under load.
- Corner half-cycle, **no pause between strides**: up to three phases, and the stance corners push through
  all of them in proportion to their time, so the body keeps moving:
  - **air** (`WALK_SWING_MS` 200, was 400 - X 2x faster): swing legs lift (Y +`WALK_LIFT_US`) and swing X
    to the start of their next stroke at the same time;
  - **down** (`WALK_LIFT_MS` 200): swing legs lower (front feet to `WALK_FRONT_APPROACH_US` above the ground);
  - **touch** (`WALK_FRONT_TOUCHDOWN_MS` 300): front feet set down slowly; handover.
  The corner-leg knee strokes (750-1100 us) run over air + down - too far for a 55 kg servo in 200 ms.
  About 700 ms per half-cycle at full speed.
- The walk runs as a **timeline** (`Timeline` in motion.cpp): every joint has its own timed segments
  (start, end, target, easing) played together each 20 ms frame - that is what lets the middle legs step at
  a different rate from the corners.
- **Easing on every move** (`EASE_FRACTION` = 0.2): a joint speeds up over the first 20% of a ramp where it
  starts and slows over the last 20% where it stops or changes direction. A joint that carries on the same
  way into the next ramp (stance legs pushing through air -> down -> touch) is not slowed at the join.
  Front touchdowns use a full soft slow-down (`EASE_SOFT`). Applies to stand / sit / walk-pose moves too.
- **Speed** (`motion::setSpeed`): every walk timing is divided by it. Console walks run at 1; the
  controller sets it from how far the stick is pushed - `WALK_MIN_SPEED` = 0.4 just past the dead zone up
  to 1 at full push, updated every half step. D-pad = full speed.
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
- **Turning, the back corners stride around a base shifted `WALK_TURN_REAR_SHIFT_US` = 150 forward**, so
  their tail-most point is the walking base (320). Unshifted they reached 470 toward the tail, hit the
  battery and browned out the ESP32.
- **Controller: release = pause in place** (2026-10-05). The moment the stick is released (or turned to a
  new direction, or the controller drops) every joint stops mid-step and holds - no finishing the step, no
  return to the stand pose. The legs stay in the **walk pose**; pushing the stick again replays the
  interrupted half-step from wherever the legs are. A change of direction starts the new gait from there:
  swing legs set all three joints every step, so the gait settles back into shape within a step. **A** from
  the walk pose goes back to the stand pose properly (`endWalk`: each group lifts, places, lowers); **B**
  sits. If it pauses with a tetrapod group in the air it simply holds on the other four.
- Console: `walk 4` = 4 full cycles; `walk` alone = until Enter. Stopping **finishes the current step**,
  then returns to the stand pose. A console walk also carries on from a paused controller walk.
- Starts from the stand pose or the walk pose; otherwise it says to run `stand`. `limp`, `stand` and `sit`
  forget the walk pose.

| Constant | Value | Meaning |
|---|---|---|
| `WALK_MID_KNEE_US` | -150 | middle-leg knee in the walk pose (was -50: feet brought in) |
| `WALK_MID_CYCLES` | 2 | middle-leg step cycles per corner half-cycle (1 = in step with the corners) |
| `WALK_CORNER_KNEE_US` | -200 | corner-leg knee in the walk pose |
| `WALK_FRONT_SPREAD_US` | 400 | FL FR X base toward the head (~54 deg; was 220) |
| `WALK_FRONT_REACH_KNEE_US` | 260 | front-leg knee at full reach (was 350: too far forward, back 25%) |
| `WALK_FRONT_REACH_Y_US` | -215 | front-leg Y when the reached foot is set down (-250 at reach 350; scaled with the shorter reach - check the toe still lands) |
| `WALK_FRONT_PULL_KNEE_US` | -750 | front-leg knee at the end of the pull (was -550; FR knee limit is -768) |
| `WALK_FRONT_PULL_Y_US` | 40 | front-leg Y at the end of the pull (was 10) |
| `WALK_FRONT_APPROACH_US` | 100 | front feet stop this far above the set-down Y at normal speed ... |
| `WALK_FRONT_TOUCHDOWN_MS` | 300 | ... then touch down over this, eased (handover runs here too). Fast drops outran the servos and punched the toes |
| `WALK_FRONT_HANDOVER_US` | 60 | handover: as one front foot lowers, the planted one eases Y up this much (ease-out, same ramp) so the body settles onto the new foot - user's idea |
| `WALK_BACK_SPREAD_US` | 320 | BL BR X base toward the tail (~43 deg; 400 scraped the battery sides) |
| `WALK_REAR_TUCK_KNEE_US` | -550 | back-leg knee when set down, start of the push (was -300) |
| `WALK_REAR_TUCK_Y_US` | -20 | back-leg Y when the tucked foot is set down (was -60) |
| `WALK_REAR_PUSH_KNEE_US` | 200 | back-leg knee at the end of the push (was -50: knee travel doubled to 500 us) |
| `WALK_REAR_PUSH_Y_US` | -140 | back-leg Y at the end of the push (was -100) |
| `WALK_BACK_MID_SPREAD_US` | 60 | BML BMR X base toward the tail (middle-leg clearance) |
| `WALK_LIFT_US` | 350 | Y up from the stand pose during swing (was 150: only ~10 mm clearance) |
| `WALK_FRONT_LIFT_US` | 500 | same, for FL FR only (was 300) |
| `WALK_STRIDE_MID_US` | 75 | middle-leg X each way from base per middle step (150 at 1 cycle) |
| `WALK_FML_EXTRA_REACH_US` | 23 | FML only: forward end of its step further (46 at stride 150, halved with it) |
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
much higher to step over obstacles. Back pair then eased to 320: at 400 BL/BR scraped the battery.
Then: front K stroke 750 -> 1100 us (reach +150, pull end +200), FML extra reach doubled to 46, and the
front feet **ease out** on touchdown (quadratic deceleration in the lower ramp, same 200 ms) - they were
stamping. (Now `EASE_SOFT` in the per-joint easing, see above.) Turns: corner X at base + stride can now reach the X limit (977 us)
and clamp there.

## Climb - `climb` (controller left trigger)

**Climb is triggered by the controller's left trigger** (LT past half travel, from the stand pose) or the
`climb` console command. The ToF sensor no longer starts a climb (`TOF_AUTO_CLIMB` = false).

**The sequence itself is not written yet** - `motion::climb()` in `src/motion.cpp` is a stub that prints
and returns; the building blocks (`Pose`, `placeGroup`, `rampType`) and `cfg::CLIMB_*` constants are
listed in its TODO. It must start and end in the stand pose so walking can resume.

## ToF sensor: higher front step

A forward-facing VL53L0X (docs/hardware.md; own I2C bus on GPIO 17/18) is sampled every 50 ms from
`loop()` and between walk steps. While it reads closer than **`TOF_NEAR_MM` = 300** (until it passes
`TOF_NEAR_CLEAR_MM` = 350 again), the front legs lift an extra **`WALK_FRONT_OBSTACLE_LIFT_US` = 600** (clamped at the Y limits) on
every step, so they step up onto / over what is in front. It acts on the next front-leg swing only -
nothing changes while standing or paused. (Was 30 mm: at the VL53L0X's minimum range it never triggered.) With `TOF_AUTO_CLIMB` = true the old behaviour returns
(flag `CLIMB`, walk stops, `loop()` runs `climb`).

| Constant | Value | Meaning |
|---|---|---|
| `TOF_NEAR_MM` | 300 | closer than this: front legs lift higher (was 30 - never triggered) |
| `TOF_NEAR_CLEAR_MM` | 350 | back to normal lift above this |
| `WALK_FRONT_OBSTACLE_LIFT_US` | 600 | extra front lift while near (was 200). With the normal 500 it passes the Y limits (FL 732, FR 2500 us), so it clamps there: ~1.6x the normal lift in practice |
| `TOF_MAX_MM` | 2000 | beyond = out of range |
| `TOF_PERIOD_MS` | 50 | sample interval |
| `TOF_AUTO_CLIMB` | false | true = ToF raises `CLIMB`, stops the walk and runs `climb` |
| `CLIMB_LIFT_US`, `CLIMB_RAMP_MS` | 300, 400 | placeholders for the climb sequence |

## Xbox controller

`walk` also takes an optional keep-going check: the controller (docs/controller.md) walks while the stick
or D-pad is held and stops the same way a key does - the current step finishes, then the legs go back to the stand pose.

## On boot

`BOOT_PULSE_US` (1500) on all 32 outputs, then if `BOOT_STAND` is true: 3 s countdown (any key cancels)
and the stand sequence. `BOOT_STAND` is **true** (enabled 2026-10-03 after the tuned stand was verified).

All values: `include/config.h`.

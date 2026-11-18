# Open a crown-capped bottle on a mounted bottle opener

A full 275 ml long-neck glass bottle with a red crown cap stands in front of the right arm. Further forward a Starr X
wall-mount bottle opener is screwed to the front of a dark wooden post, as behind a bar. A cork coaster lies near the
middle of the table.

The instruction is: "Open the bottle on the mounted bottle opener, then stand the open bottle on the coaster and let go."

Bartenders and servers open crown-capped bottles on fixed openers many times a shift. The opener can't move, so the
bottle has to be brought to it and used as the lever:
- The bottle is tipped well over, top toward the wall, and its cap is pushed up into the opener's mouth until the cap's
  top meets the fulcrum pad.
- It is moved so the prying lip hooks under the edge of the cap's skirt.
- The bottle is pressed down toward upright. It pivots on the pad, the lip holds the skirt's edge, and the crimp is
  pried open.

A crown cap can't be pulled or twisted off by hand. Prying it takes a real moment (about 0.9 N m), which a hand on the
bottle's body, 12 cm from the pad, supplies by pressing with about 7.5 N. Then the open bottle is stood on the coaster.

## Revision (after TK-VPVRG6)

| Review point | What changed |
|---|---|
| Construct validity: once caught, the cap moved and popped with no crimp resistance | The crimp is now a physical joint between cap and bottle (`CRIMP` in `task.py`, `TiltJoint` in `partkit.py`). The cap can't slide or turn on the finish; it can only tilt off it, against a spring of 8.6 N m/rad. The crimp lets go for good when the moment on the cap reaches 0.9 N m, a real 26 mm pry-off crown's opening torque (see [The crimp](#the-crimp)). The robot has to deliver that moment through the bottle against the opener. The scripted pop (a 1 cm jump), the rule that put the cap back on the finish, and the 1.5 mm "pop on bend" rule are gone. |
| ... then a full-score demo with that resistance | The opener is now a named, sold product, a Starr X, modelled from its maker's patents (see [The opener](#the-opener)). The demo uses it the way people do, holding the bottle by its body as the lever. Holding the lower neck can't supply the moment: the lever from the neck to the pad is 3.2 cm, which would need 28 N from a wrist rated 7 N m (see [Feasibility](#feasibility)). The demo opens it at step 309, with 0.94 N m on the cap. |
| Scoring: Isaac Lab link quaternions passed to an xyzw function | This runtime is Isaac Lab 3.0, the version yam_arena runs on. Isaac Lab 3.0 orders its pose quaternions (x, y, z, w); only 2.x used (w, x, y, z). yam_arena's own link-pose and wrist-camera code read them that way. The standing bottle, read from the same buffers, scored as upright in every earlier run. To take the question out of the scorer altogether, the let-go check no longer reads any link quaternion. It poses every arm link from the arm's measured joint angles, using the URDF's forward kinematics (`_link_poses`, `arm_link_poses`). It then checks that each link's position is within 5 mm of the simulator's own; if not, the bottle never counts as let go. At reset, a bottle that reads upside down stops the trial with an error. |
| Test the live scorer held at the neck, at the body, and released | `test_task.py` runs the scorer's runtime path, `_follow` then `released()` and `score()`, on stand-in simulator data in Isaac Lab's layout. The arms are posed for each case: neck held, body held, body touched, and let go. It gets 0.75, 0.75, 0.75 and 1.0, the same whichever order the link quaternions are given in. Given (w, x, y, z), the previous version gave 1.0 to the neck hold. In the simulator, the run set has a variant for each case, and the gaps the live scorer measured are in [Let go](#let-go). |

Earlier rounds asked for a real bottle and crown, a complete and placed opener, full credit only when nothing touches
the bottle, and documented positions that match the model. Those all still hold. The positions below are the model's.

## The bottle and the cap

All sizes are in the model (`task.py`) and in `task.json`. Positions are in cm from the midpoint between the arm bases
(x forward, y left), heights above the table.

- **Bottle:** a 275 ml long-neck glass bottle with a standard 26 mm crown finish, 450 g filled with still water
  (beer weight, but nothing foams over a shared rig), standing at x 27, y -25. It is 18.5 cm tall to the top of the finish. The body is 5.8 cm across and
  11 cm tall. The shoulder narrows to a lower neck 3.2 cm across (13 to 16.2 cm up), then to 2.35 cm under the finish.
  The finish's bead is 2.64 cm across and 4.3 mm tall.
- **Crown cap:** a 26 mm pry-off crown, crimped on: 2.88 cm across the skirt and 6 mm tall. Its skirt reaches 0.7 mm
  below the bead, and its lower edge stands 1.2 mm out from the bead all round. That edge is what the prying lip gets
  under.

## The opener

A **Starr X wall-mount bottle opener** (Brown Mfg. Co., Newnan, Georgia), the classic zinc-plated cast-iron bar opener
that has been made since 1925 and is still sold. Spill-Stop sells the same pattern in die-cast zinc as part 13-300. It
is about 8.3 cm tall, 7 cm wide across its upper face and 3.2 cm deep (3-1/4 x 2-3/4 x 1-1/4 in, as listed), and is held
by two screws. The model is 8.3 cm tall, 7 cm wide and 3.4 cm deep.

- **How it works (the maker's patents):**
  - US 1,534,211 (1925): the fulcrum presses only on the middle of the cap, a lip engages under the edge of the cap, and
    the bottle is tipped on the fulcrum to take the cap off.
  - US 2,333,088 (R. M. Brown, 1943, "Bottle opener"): the frame juts out from the wall as a salient angle. The capped
    mouth goes up through an opening in its lower face. A fulcrum pad presses on the middle of the cap's top. The
    opening's outer edge (the centring member) runs back as a ledge whose upturned edge is the prying lip, catching the
    base of the cap's skirt. Back stops stand above the lip, spaced from it so the skirt can flare. The bottle is
    "depressed as a lever" on the pad.
- **The model's working parts** follow Fig. 2 of the 1943 patent, sized to a 26 mm crown. The bottle sits in the mouth
  tipped 53 degrees from upright, as drawn there. In the bottle's own frame, with the cap's top on the pad:
  - **Pad nose:** 6 mm wide, over the middle of the cap's top.
  - **Prying lip:** reaches in to 13.5 mm from the cap's axis (0.9 mm under the 1 mm skirt wall, 0.3 mm outside the
    bead), 1 mm beyond the skirt's edge.
  - **The mouth's edge:** 0.5 mm off the skirt on the lip's side. It stops the cap when it is moved over onto the lip.
  - **Back stops:** 3 mm off the skirt above the lip.
  - **The mouth's sides:** 1.5 mm off the cap.
  - The rest of the casting (the base on the wall, the upper face, the screws) is plain boxes, as is the post.
    Shapes that are only seen, with no collider or mass, round it off for the cameras: the rounded ears round the
    screws and the hood's rounded nose.
- **Placement:**
  - The opener's back is on the front face of a dark-stained hardwood post, 9 cm wide and 2.5 cm deep, at x 41.5. The
    post stands on the table, is clamped to it, and runs up to 23.6 cm.
  - The opener's mouth faces down and toward the robot, centred on y -20. Its screws are 14.9 and 21.3 cm up, and it
    spans 14.0 to 22.3 cm up.
  - With a cap in the mouth, the middle of the cap's top is at x 40.0, 17.6 cm up.

## The crimp

A crimped crown holds the bottle's gas pressure, so it can't be pulled or pushed off. It comes off when one side of the
skirt is pried away while the top is held: the flutes on that side open out over the bead and the cap tips off.

- **Model:** a PhysX D6 joint between cap and bottle (`CRIMP`). It holds the cap's origin at the middle of the finish's
  top, with no sliding and no turning about the bottle's axis. Tilting off the axis is resisted by a spring:
  `CRIMP_K` = 8.6 N m/rad, and 0.005 N m s/rad of damping. Cap and bottle don't collide while it holds.
- **Letting go:** after every control step the task file measures the cap's tilt on the finish. When the tilt reaches
  6.0 degrees (`THETA_POP`, the pried edge 1.5 mm up), the spring is carrying 0.9 N m (`T_OPEN`), and the joint is
  switched off for good. The cap is then a loose part: it drops out of the mouth or tips off the finish. Nothing pushes
  or moves it. With `BOTTLE_DEBUG=1`, the moment is printed every step while the cap is caught.
- **Opening moment, 0.9 N m (8 lbf in):**
  - A crown maker's technical data sheet for its standard pry-off crowns gives an opening torque of 4 to 9 lb in, that is
    0.45 to 1.0 N m. These are 6 mm tall crowns with 21 flutes, from 0.20 to 0.22 mm steel (Astir, "Crown cork
    specifications", technical data sheet: "Opening torque: from 4 to 9 lb / in").
  - US patent 5,458,253, "Bottle cap", reports pry-off removal torques around 7.8 to 8.1 in lb (0.9 N m) for its pry-off
    crowns.
  - The German brewers' and metal packaging associations' delivery terms for crown corks give the twist-off range,
    4 to 12 lbf in, the crimp diameter (28.6 to 28.8 mm) and the pressure the crimp must hold (at least 7 bar) (Deutscher
    Brauer-Bund and Verband Metallverpackungen, "Special technical terms of delivery and supply for crown corks", 2016,
    sections 5.4, 5.8 and 6).
  - 0.9 N m sits at the top of that range: the robot is not given an easy crimp.
- **What it doesn't model:** at 7 bar the crimp holds about 300 N straight off. No arm comes near that, so the joint
  never lets go to a straight pull or push. The skirt stays rigid; a real one bends up on the pried side as it comes off.

| | Real 26 mm pry-off crown | The model |
|---|---|---|
| Moment that pries it off | 0.45 to 1.0 N m (data sheet); about 0.9 N m (patent tests) | 0.9 N m (`T_OPEN`) |
| Pried edge's lift when it lets go | 1 to 2 mm, the flutes opening over the bead (an estimate, not measured) | 1.5 mm (`POP_LIFT`): a 6.0 degree tilt on the finish |
| Straight pull to take it off | at least 7 bar of gas pressure, about 300 N | never: the joint doesn't let go to a pull |
| Twisting it off | not possible by hand (pry-off crowns have no thread) | not possible: no turn about the axis |
| Force on the bottle to open it on this opener | 0.9 N m about the pad: about 7.5 N on the body 12 cm from it | the same, by construction |

### Feasibility

The arm is soft: its joints are springs of 80 N m/rad (shoulder and elbow) and 10 N m/rad (wrist), with no compensation
for what it holds. It is limited to 27 N m at the shoulder joints and 7 N m at the wrist. A pinch also turns easily
about the axis the fingers close along.

These numbers come from the URDF's kinematics, at the moment the crimp lets go: the bottle pressed 9.5 degrees from
where it went in, 43.5 degrees from upright. The scratch scripts aren't part of the task.
- **Lever:** the hand holds the body 12 cm from the pad.
- **Push:** 0.9 N m needs 7.5 N there, pressing the body toward upright.
- **Arm:** its stiffness that way is 550 N/m (1.4 cm of command overshoot). The joints are at no more than 0.17 of their
  rated torque.
- **Lower neck instead:** holding the lower neck (revision 3's grip) leaves only 3.2 cm of lever. That needs 28 N from
  a wrist rated 7 N m, and the pinch would turn about its closing axis.

On a real rig this fits the YAM too:
- **Grip:** the fingers close at up to about 40 N each. At a friction coefficient of 0.5 between the pads and the glass,
  the pinch holds 40 N along the body. It has to carry the bottle's weight (4.4 N) and the 7.5 N push, 8.7 N together.
- **The opener's forces:** the pad and the lip squeeze the cap with about 55 N each, 17 mm apart. That squeeze stays
  between the casting and the cap and never passes through the fingers.

The demo's press is commanded 20 degrees in all, 2.2 cm past where the crimp lets go. The arm's give makes that about
12 N, or 1.45 N m about the pad, 1.6 times what the crimp needs. The press only turns the bottle about the pad: one that
also moved it on past would drive the open bottle into the post. The full bottle makes the loaded arm sag about 3 cm. The demo
aims the other way, using a static model of the joints' sag under the bottle's weight, and a probe run in the simulator measures what is left
over (`BOTTLE_FIX_OPENER`, `BOTTLE_FIX_COASTER`).

## How it differs from nearby tasks

- **Paint-tin lids pried off with a hand tool** (a lid lifter's claw hooked under tabs, or a screwdriver levered under
  the lid). There the container stays put and a hand-held tool does the work. Here the tool is fixed, and the robot
  carries the workpiece, a full bottle tipped well over in mid-air, into it and levers it against the tool.
- **Uncorking a wine bottle with a corkscrew:** a hand-held tool is screwed in and pulled while the bottle is held still.
- **Twist-off, flip-top, child-resistant and latched bottle caps:** these are opened by hand, with no tool.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects (the Starr X model included), the crimp joint, the let-go check, `score()`, `success()` and the scripted `demo()` |
| `partkit.py` | Rigid parts built from boxes and cylinders (fixed ones too), joints between them (`TiltJoint`), pose writes and step/reset hooks |
| `check_release.py` | The let-go test on kinematic arm poses round the bottle, without the simulator (yam_arena and NumPy) |
| `test_task.py` | Offline tests of the scorer's runtime path on stand-in simulator data: `python -m pytest test_task.py` |

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this
folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Open the bottle on the mounted bottle opener, then stand the open bottle on the coaster and let go." \
  --scorer task_score --no-prompt --max-steps 1200 --store-frames --log-dir ~/yam-eval/bottle-opener-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. The layout is
fixed (`object_jitter=0`), so no seed is needed. A trial takes about 3.5 minutes of wall time on an L40S at 1,200 steps.

## Scripted demo

Run it with `--policy yam_scripted` instead of `--policy dream`, and `--max-steps 750`. It reaches full score. The crimp
lets go at step 309, with 0.94 N m on the cap and the bottle 44 degrees from
upright. The episode ends on success at step 544 (27.2 s of simulated time). Set
`BOTTLE_DEBUG=1` to print the bottle, the cap and the trial record every 5 steps, and the crimp's moment every step
while the cap is caught.

1. The right hand comes at the bottle from the robot's side, its approach tipped 53 degrees from vertical toward the
   post. It closes across the body, 6.5 cm up, and lifts the bottle 3.5 cm.
2. It tips the bottle over 53 degrees, top toward the post, so the hand ends up pointing straight down. It carries the
   bottle under the mouth and pushes the cap up into it, beside the lip, until the cap's top is on the pad. Then it
   moves the cap over until the skirt meets the mouth's edge, which puts the lip under the skirt's edge.
3. It presses the bottle down toward upright about the pad, 20 degrees in all. The crimp's moment builds up to 0.9 N m
   and the crimp lets go, about 9.5 degrees into the press.
4. It backs the open bottle out of the mouth, turns it upright, stands it on the coaster, lets go, backs the hand off
   along its approach and parks it.

Meanwhile the left hand only carries its wrist camera beside the opener, so the video shows the pry. From the top
camera the right arm hides it. Before the cap goes in, the left hand comes to rest with the camera 27 cm from the lip,
looking into the mouth from the post's left and a little below. Through the gap under the mouth's left side it sees
the lip's edge under the skirt and the skirt's edge beside it, and then the cap as it falls. The mouth's sides hide
the rest of the cap while it is in the mouth.

The hand goes back to its ready pose before the bottle is carried to the coaster. It stays at least 14 cm from the
opener and the post, and at least 9 cm from the bottle and the right arm.

Variants, with `DEMO_VARIANT` set in the environment:

| `DEMO_VARIANT` | What the arm does | Score |
|---|---|---|
| `nothing` | waits | 0.0 |
| `skip_opener` | carries the closed bottle straight to the coaster and lets go | 0.0 |
| `pull_cap` | pinches the cap and pulls it straight up: the crimp holds, and the bottle comes up with it | 0.0 |
| `no_hook` | pushes the cap up onto the pad but never moves it onto the lip, then presses down | 0.0 |
| `pry_short` | hooks the lip and presses the bottle only 4.5 degrees, just onto the lip: the crimp's moment peaks at 0.21 N m and it holds | 0.25 |
| `no_place` | opens the bottle and keeps holding it | 0.5 |
| `no_release` | opens it and stands it on the coaster, still holding its body | 0.75 |
| `neck_hold` | opens it and stands it on the coaster, then loosens its fingers to 4 mm off the glass and slides its hand up the bottle, the fingers following the shoulder. It closes on the straight part of the lower neck, above the shoulder's taper, and presses the bottle down onto the coaster. The hand is never 1 cm clear of the glass on the way, so it never counts as let go | 0.5: held to the end, so never let go and never 1.0, but the closing hand pushed the bottle about 3 cm off the coaster's centre (the limit is 3 cm), so it is no longer on the coaster (designed for 0.75) |
| `touch_body` | opens it, stands it on the coaster, lets go, and rests one open finger's pad on its body | 0.75 |
| `probe` | carries the bottle under the mouth and over the coaster and holds it still (measures the arm's misses) | - |

## Scoring

The task file follows the trial after every control step and keeps a record that is cleared on reset:

- **Caught:** the prying lip is under the edge of the cap's skirt. This is measured in the cap's own frame, since the
  cap tips as it is pried. The lip's edge is 12.8 to 14.2 mm out from the cap's axis (under the skirt's wall, outside
  the bead) and 0 to 2.5 mm beyond the skirt's edge (-0.5 mm allows for contact). The cap's axis is within 6 mm of the
  mouth's centre line sideways.
- **Opened:** the crimp let go (see [The crimp](#the-crimp)) while the cap was caught, or within 5 steps (0.25 s) of it.
  If the crimp is pried open anywhere else, the trial scores 0.
- **Tipped:** the bottle tilted more than 65 degrees at any time, or fell off the table. The opener takes it, closed,
  tipped 53 degrees; it comes out at about 33.
- **Let go:** once it is open, after every step: no part of either arm is within 1 cm of any part of the bottle. See
  [Let go](#let-go).

`score()` gives 0 if the bottle tipped, or if it was opened off the opener. Otherwise it gives:

- **1.0** when the bottle was opened on the opener and stands on the coaster, and three things hold for 1 s:
  - it is on the coaster: its axis within 3 cm of the coaster's centre, its base on it, and within 10 degrees of
    upright;
  - it is still: moving under 1 cm/s and turning under 0.5 rad/s;
  - it is let go: no part of either arm (fingers, gripper, wrist or arm) is within 1 cm of any part of the bottle, body
    included.
- **0.75** when opened and on the coaster, but still held, touched or moving.
- **0.5** when opened.
- **0.25** when caught.
- **0** otherwise.

`success()` ends the episode once 1.0 has held for 1 s.

### Let go

Full score needs both arms clear of the whole bottle, not just of the place where the demo grips it. A hand that holds
the bottle by its neck or body, leans on it, or keeps a finger on it gets 0.75.

- **The arms:** every moving link of both arms (`link1` to `link5`, `gripper`, `tip_left`, `tip_right`), as the convex
  collision hulls the simulator collides with. These are the YAM URDF's CoACD hulls, which the arm's USD is converted
  from unchanged. Their surfaces are sampled once, at reset: every 3 mm on the gripper, fingers and wrist, and every
  6 mm on the upper and lower arm.
- **Where the arms are:** each arm's measured joint angles (`joint1` to `joint8`) run through the URDF's forward
  kinematics, from its base link's position. The simulator's links follow the same kinematics; yam_arena checks the two
  against each other. No pose quaternion is read. Each link's position from the kinematics is also checked against the
  simulator's own; if any is more than 5 mm off, the bottle never counts as let go.
- **The bottle:** the exact glass the model is built from (`BOTTLE_PROFILE`): body, label band, shoulder, necks and
  finish, as stacked cylinders in the bottle's own frame.
- **The test:** the sample points are moved to the links' poses, and their signed distance to the bottle is computed
  exactly. Every hull point lies within 0.6 × the spacing of a sample, so each link's distance is reduced by that much
  first. The bottle counts as let go when the result is at least 1 cm, so an arm part 1 cm or closer never passes.

`python check_release.py` runs the test on kinematic poses round the bottle standing on the coaster (fingers that hold
it are posed with their pads on the glass, where the simulator stops them):

| Case | Nearest part | Gap | Let go |
|---|---|---|---|
| Right hand holding the lower neck from above | right `tip_left` | -1.8 mm | no |
| Right hand holding the body (the demo's grip) | right `tip_left` | -2.2 mm | no |
| Right hand's open pad resting on the body | right `tip_right` | -4.2 mm | no |
| Right fingers just opened round the body | right `tip_left` | +7.8 mm | no |
| Right hand backed off 8 cm along its approach | right `tip_left` | +38.0 mm | yes |
| Left fingertips 6 mm off the body, 7 cm up | left `tip_right` | +4.9 mm | no |
| Left fingertips 2 cm off the body, 7 cm up | left `tip_right` | +19.8 mm | yes |

The live scorer in the simulator, at the end of each run (gap from the nearest arm part to the bottle):

| Run | Gap | Score |
|---|---|---|
| `neck_hold`: slid up from the body and held by the lower neck | -1.9 mm | 0.5 (held, so never 1.0; the hold had pushed the bottle off the coaster's centre) |
| `no_release`: held by the body | -1.8 mm | 0.75 |
| `touch_body`: one pad resting on the body | -1.9 mm | 0.75 |
| The demo: let go | +28.8 mm | 1.0 |

## Real-world build

- **Bottle:** a 275 ml long-neck glass bottle with a 26 mm crown finish, filled with still water and capped with a
  standard 26 mm pry-off crown on a bench crown capper. Homebrew suppliers sell the empty bottles, the crowns and the
  capper; each trial uses a freshly capped bottle.
- **Opener:** buy a Starr X wall-mount bottle opener (Brown Mfg. Co.), or the same pattern sold as Spill-Stop 13-300.
  Screw it with its two screws to the front of a hardwood post 9 cm wide and 2.5 cm deep, clamped to the table, with
  the post's front at x 41.5 and the opener centred on y -20. Mount it mouth down, its upper screw 21.3 cm and its lower
  screw 14.9 cm above the table.
- **Scoring:** from a video and a ruler. To judge the 0.25 level and that it was opened on the opener, film the mouth
  from the side (the lip under the skirt's edge) and from above (cap centred in the mouth).
- **Safety:** opening the bottle is safe. The arm presses the bottle a few centimetres with the cap in the opener's
  mouth. The bottle holds still water, so nothing foams or spills onto the rig.

## Licence

This task is released under the MIT licence. `partkit.py` comes from `tasks/din-rail-mount` in this repository, with
fixed parts, cylinder axes and a joint between parts (`TiltJoint`) added, under the same licence. No third-party models
or assets are used. Every object, the opener included, is built from primitive shapes.

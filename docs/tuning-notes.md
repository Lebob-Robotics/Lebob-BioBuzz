# Tuning notes

The measurements and history behind the code's constants and rules: the dates, log names, counts and earlier
values that used to sit in the code comments. The comments keep the reason; this file keeps the evidence.
Grouped by area, then by class, then by constant or method name. A six-digit number such as 211957 is the
time (HHMMSS) in a PsiKit log's or a tuning session's name on that day. The logs stay outside this repo.

## Shooting and feeding

From `shooting/` and `shooting/feed/`.

### AimConstants (shooting)
- `FEED_DELAY_S` (0.30): the flywheels' RPM dipped 0.12-0.30 s after the fire command, mostly about 0.2 s (nine feeds, 4-5 Nov logs). With one ball per pulse, the dip came 0.20-0.35 s after the pulse. At 0.20, two strafing shots clipped the Cell's edge on the side the robot was moving toward (6 Nov, 080732).
- `AIM_VELOCITY_SMOOTHING_S` (0.12): unsmoothed, the sideways speed's noise swung the aim heading 2.4-3.4 degrees rms during passes and the lock chased it (7 Nov, 100030).
- `AIM_AT_FRONT_CELL` (true): the tags' up-Cell vote flipped 2-7 times a run on 13 Nov, and the far volleys went to the far Cell, past and right of the near one the driver meant. Decided as the driver's call on 14 Nov.
- `HOODED_RPM_OFFSET` (0): was -75 (6 Nov) to make up for the model's ball being too slow; the model was refitted to Shot Tuning's 224 hooded balls (7 Nov).
- `HOODED_HEADING_TRIM_DEG` (2.0): Shot Tuning's aim search settled at it in session 085918. SOTM Tuning's 16 finished hooded spots (7 Nov) put the hooded trim at +1.92 far, +2.01 mid, +1.85 near. TeleOp previously had no trim.

### BackwardShotConstants (shooting)
- `BACKWARD_DIST_M` / `BACKWARD_RPM` / `BACKWARD_HOOD_DEG`, 0.40 m point: from Shot Tuning, 6 Nov 082517. On its stop the hood never scored from 0.4-0.6 m (0 of 7 at 3400-3600 rpm, and the G1 grid 0 of 3 at 0.6 m, onto the Cell's bottom); 8 degrees up at 3200 rpm scored 4 of 5, and 3150-3300 rpm scored 8 of 13. Shot Tuning 091010: 0.42 m at 90.3 degrees 8 of 10 against 0 of 2 at 91.3. At 0.40 m, 88-94 degrees all scored at 3200 rpm but none often (14 of 52 over session 085918), so 90 stayed.
- `BACKWARD_*` tables, 0.60-0.85 m results: Shot Tuning 091010: 0.60 m at 3270 rpm and 94.0 degrees scored 5 of 7; 0.72-0.74 m as interpolated (3310-3320 rpm, 95.5-95.9 degrees) scored 14 of 25, misses unmarked. Shot Tuning 080751 (7 Nov): 0.77 m as interpolated (3320-3330 rpm, 95.9-96.5 degrees) scored 4 of 4, 23 degrees off the Cell's axis; 47-51 degrees off the axis, 0.74-0.85 m scored 3 of 14 at 3370-3470 rpm and 94.7-97.7 degrees.
- `BACKWARD_*` tables, 0.63-0.81 m points: Shot Tuning 085918 (7 Nov), each spot's RPM search at the old table's angle, the middle of what scored: 0.63 m 3000-3360 rpm at 94.3 degrees (7 of 7 at 3180); 0.73-0.74 m 3200-3340 and 3020-3350 at 95.8-95.9; 0.76 m 3280-3310 at 96.5 (the line between 0.73 and 0.81 gives 3300 there); 0.81 m 3350-3420 at 97.0.
- `BACKWARD_*` tables, 0.95 m out: set 13 Nov. The 3 Nov one-ball points went long as volleys in the wedge. Lebot Teleop's 8 far backward balls (logs 173844-225900, hood 97.6-98.2) left at 3505 rpm from 1.53 m, 3566-3597 from 1.52-1.59 m, 3628 from 1.85 m and 3782-3812 from 1.94 m, all long. The drag model was fitted to Shot Tuning's 179 marked backward balls (0.4-0.9 m). Resulting points: 1.53 m 3411 rpm (was 3592), 1.85 m 3494 (was 3750), 1.94 m 3555 (was 3836). The 2.3 m point is the model's shape scaled to 1.85 m.

### FireConstants (shooting)
- `RPM_TOLERANCE` (125): the wheels settled a step or so above target, 3605 read as 3628-3689 (4 Nov); at 75 the shot was held back about 3 s.
- `RPM_MEAN_ABOVE_TARGET` (50) / `RPM_MEAN_BELOW_TARGET` (100): both misses of 5 Nov (080106) left 56 and 80-98 RPM fast, one of them long; balls 63 RPM slow went in.
- `RPM_PREDICTED_ABOVE_TARGET` (100): wheels climbing back after a ball were fed at +17 RPM and the ball left at +130, long (5 Nov, 080706). Balls predicted +61 and +85 went in.
- `MAX_HEADING_TOLERANCE_DEG` (4): the misses were 7.7 and 13.6 degrees off (5 Nov, 083109-092330). 4 keeps 92% of moving READY time (soak 081210, median 1.6).
- `MOVING_SHOT_SPEED_MPS` (0.3) / `MIN_MOVING_SHOT_LIP_M` (0.7) / `MAX_HOOD_RATE_TO_FIRE_DEG_PER_SEC` (12): strafing past at 0.7-0.8 m/s, balls fed READY at 0.62 and 0.48 m both missed; during the 0.3 s feed the heading error went 7.7 -> 20.8 degrees and the hood target moved 6 degrees. The standing shot from 1.5 m went in (5 Nov, 083109).
- `AIMED_READY_S` (0.0): waiting two or three loops cost more shots than it saved while moving (5 Nov, 084358).
- `AIMED_FEED_PULSE_S` (0.30): the webcam saw first balls leave 0.15-0.28 s after the feed began and second balls 0.13-0.4 s behind them (5 Nov, 084355-092330).
- `AIMED_FEED_PAUSE_S` (0.35): at 0.25 s the third ball's dip came just after the pause (6 Nov, 080814).
- `MIN_BALL_GAP_S` (0.35): a ball was refed 0.08 s after the last (5 Nov, 084355).
- `FEED_COMMIT_S` (0.4): a ball fed standing still left 0.36 s later with the robot at 0.73 m/s and turning 44 deg/s after the stick was pushed, and missed right (5 Nov, 084758).
- `FEED_COMMIT_MAX_S` (1.0): back-to-back feeds renewing the hold ran the robot into the fence (5 Nov, 090446).
- `FIRE_STEADY_STICK_S` (0.3) / `FIRE_STICK_CHANGE` (0.1): the ball fed in the loop the stick was pushed left with the robot at 0.56 m/s, the wheels 250 RPM fast for that speed, and went long (5 Nov, 085458).
- `MAX_ACCEL_TO_FIRE_MPS2` (1.0): costs 8% of moving READY time (soak 081210).
- `ACCEL_WINDOW_S` (0.2): over 0.1 s, driving at 0.6+ m/s on a stick held still 0.6 s, 12% of loops read over 1 m/s^2 (median 0.54), and the gate held 40-46% of RT time at 0.6-1.2 m/s. Over 0.2 s, 2% of loops (13 Nov).

### ShootOnTheMove (shooting)
- `approximateTurningRad`: turning 0 instead jumped the aim heading 25 degrees and back while strafing (5 Nov, 094211).
- `aimHooded` (shot now calculated, not taken back by its rate): rate noise times the feed delay swung the hood target 6 degrees loop to loop (5 Nov, 082111).
- `inFrontOfCell`: until 14 Nov it took "out" as toward the field's middle, from Cells once thought to be on the walls, so the wedge never held and TeleOp shot hooded from straight in front (230333: 11.5 degrees off the axis, the balls bounced out).
- `stopHint`: on 13 Nov 8 presses (17 s) never fired, strafing at 0.5-1 m/s 0.35-0.7 m from the lip.
- `tooCloseToShootMoving`: the shots it refuses missed on 5 Nov.

### ShooterGeometry (shooting)
- `HOOD_TOP_STOP_DEG` (54.3): the second hard stop, measured 5 Nov (HoodTune 083632, lift 44.7). The first hard stop was 58.5; 51 before the stops were fitted.

### ShooterTuning (shooting)
- `RPM_OVERRIDE` (0): was 4440 before the shot table was calibrated.

### ShotPickConstants (shooting)
- `BACKWARD_PICK_MAX_DIST_M` (0.80): standing (6 Nov), backward from 0.42-0.60 m scored 13/17 against hooded about 50%; the drivers chose 0.72 m as the switch. Moved to 0.80 (7 Nov) so the 0.77 m shots the drivers take backward (4 of 4, Shot Tuning 080751) are backward. Hooded at the table's settings scored 46 of 55 at 0.75 m over the same sessions, against 18 of 29 backward.
- `BACKWARD_GIVE_UP_S` (1.0): driving around the Cell at full stick held a backward pick for 30 s with no shot (5 Nov, 094211).
- `SHOT_PICK_QUICKEST` (true) and the `SHOT_PICK_*` rates: quickest pick since 8 Nov. Fitted to Lebot Teleop's logs (1-7 Nov, 40 logs): 58 heading-lock turns, median 0.50 s for 10-30 degrees, 0.77-0.92 s for 30-90 and 1.15 s for 90-180; 128 hood moves, median 0.23 s for 4-8 degrees, 0.41 s for 15-25 and 0.47 s for 25-40. The wheels: about 4000 rpm/s at full power.
- `SHOT_PICK_BACKWARD_MAX_LIP_M` (0.85): Shot Tuning had 331 backward balls at 0.4-0.8 m and 5 past 0.85 m.
- `BACKWARD_ONLY_WEDGE_DEG` (30): added 13 Nov.
- `BACKWARD_WEDGE_MAX_LIP_M` (0.95): the wedge never held until the 14 Nov fix; 61 of 427 TeleOp balls (7-14 Nov) left from 0.9-1.1 m in front, against the rear wall. The bounced 0.90 m volley it takes in was in log 230333.

### SotmConstants (shooting)
- `SOTM_HOOD_DEG_PER_TOWARD_MPS` (0) / `SOTM_RPM_PER_TOWARD_MPS` (0) / `SOTM_HEADING_DEG_PER_SIDEWAYS_MPS` (0): fitted 7 Nov to 16 finished hooded spots (0.3-0.7 m/s, 0.8-1.6 m): hood -1.22 +/- 1.02 deg per m/s toward (scatter 1.35 deg; standing error by band far +0.8, mid +0.75, near -1.6); aim -0.13 +/- 0.32 deg per m/s sideways (scatter 0.41 deg); release delay 0.292 +/- 0.031 s against FEED_DELAY_S 0.30. Left out of the fit: F ARC left 0.60 at 0.98 m, whose aim (-5.25) came from 8 balls left in a row at a +6 deg hood trim while the aim step was stuck at 0.25 deg.
- `SOTM_HOOD_TRIM_DIST_M` / `SOTM_HOOD_TRIM_DEG` ({0.95, 1.48} / {-2.3, 0.7}): all 17 found spots' hood centres: -0.88 +/- 0.18 deg at 1.2 m, +5.53 +/- 0.76 deg per m (scatter 1.29 deg; adding toward -0.64 +/- 0.52, adding |sideways| -1.92 +/- 1.37: noise). Standing balls (Shot Tuning 085918) centre -2.5..+2.4 deg either side of the table at every distance. With the trim all 16 spots' scoring windows hold the shot (worst 0.57 deg inside); without it, toward-right 0.50 at 0.91 m (window -3.7..-1.5) and field -y 0.70 at 1.14 m (-4.0..-0.5) miss.

### OneBallFeedGate (shooting.feed)
- `DIP_FROM_PEAK_RPM` (150): 312 RPM had to go before below-target counted the ball; the feed ran on 0.45 s and pushed a second ball out (5 Nov, 085458).
- `VOLLEY_DIP_RPM` (150) / `VOLLEY_PEAK_DROP_RPM` (170): volley 9 Nov 203128: 153 RPM falls, 152-183 under target, counted dry pulses as the third and fourth balls, under the old ringing recovery. Under speed control a ball's fall is 216-368 RPM (12 Nov, 213719), but a 300 peak drop missed two of the four balls, a pause and a retry each. 200 from the peak still missed 52 of 276 balls on 13 Nov, most of them the last: real balls fell 154-200.
- `OneBallFeedGate(readySeconds, stopUnreadyAfterS)`: the empty feeder ran on 11 s (5 Nov, 094738).
- `OneBallFeedGate(..., pulseS, pauseS, minGapS)`: 5 Nov video, second balls 0.13-0.4 s behind the first; a ball was refed 0.08 s after the last (5 Nov, 084355).
- `lead`: waiting for the band first made each later ball of a volley take 0.58 s against 0.39 s (13 Nov); the 13 balls fed early when a dip was missed left at the same speeds as the rest.
- `feederEmpty`: the last ball rocked at the bottom of the indexer for 10 s (6 Nov, 081243).

### RpmTrend (shooting.feed)
- class: 5 Nov, 080706: fed at +17 RPM, left at +130.

### StackPressure (shooting.feed)
- class: 13 Nov log 225900 at 73.5 s: 0.6 s at 4.4-4.8 A, then a 400 RPM dip.
- `ON` (false): turned off 15 Nov (134217); it tripped 1.1 s into loading and the intake spent 46% of its on-time at half power with the last roller stopped.
- `REARM_RELEASE_S` (0.5): drivers tap LT 25-40 times a minute (13 Nov).
- `FREE_AMPS` (2.5) / `FREE_S` (0.3): currents from 14 Nov log 230333. A ball taken in tripped "packed" there, and eased the intake then stayed at half power for 13 s, grinding balls in at 4-5 A.

### WheelContact (shooting.feed)
- class: 13 Nov, 79 such dips over 917 s of loading.
- `DIP_RPM` (300): shots fed at speed dip a median 307 RPM.
- `NEAR_TARGET_RPM` (250): overshoot example, log 191420 at 10.1 s: 3874 on a 3563 target.
- `ABOVE_TARGET_RPM` (100): the Scoring Auto's target stepped from 3942 to 3733 while it drove, the wheels fell from 3996 to 3689, read as a full stack, and it ejected at the rear Flower for 3.2 s (13 Nov 224556).

## Volleys and tuning

From `shooting/volley/`, `tuning/` and `opmodes/tuning/`.

### ShotTuning (opmodes.tuning)
- `fromBackwardTable`: Shot Tuning session 085918 (7 Nov) began a backward run at 0.90 m's launch angle from 0.64 m, because the table was loaded for a Cell guessed before the tags were seen.
- `startSearch` (the backward shot always searches the RPM): Shot Tuning 085918 (7 Nov): back G1's launch angle lowered 94 -> 88 degrees, 12 of 12 balls short.

### SotmTuning (opmodes.tuning)
- `drive` (steady reference from the plan's speed only): seen 7 Nov, 085204, when a field-frame stick on an arc never held still and the Steady gate never let an arc fire.
- `drive` (fire held until the ball is out): 7 Nov, 080123: with fire released the loop the feed started, 5 of 16 passes had no ball.

### SpecialShotCalibration (opmodes.tuning)
- `START_RPM_FAR_UP` (5110) / `START_LAUNCH_DEG_FAR_UP` (72): the first try, 4960 rpm at 72 deg (the old model's pick), missed.

### VolleyAutoTuner (shooting.volley)
- `SOLID_GOOD` (4): the inconsistent two-volley confirmations were in the 15 Nov matches.
- `completed` (good twice in a row at one setting confirms it): setting 1 (full feeder and indexer) went 8 of 8 and is what TeleOp runs. The confirmed trims are the middle of the scoring range because on 14 Nov spot 4 went 4 of 4 at trim 0 and at -1.

### VolleyConstants (shooting.volley)
- `VOLLEY_READY_BELOW_RPM` (100): a band of 75 flickered in and out. The finding that balls fed 50-100 under went in as often as balls fed on target is from Volley Tuning on 12 Nov.
- `VOLLEY_BACKWARD_ON` (true) / `VOLLEY_BACKWARD_READY_ABOVE_RPM` (25): backward volleys were added 13 Nov. Shot Tuning's 138 standing backward balls dipped the wheels median 307 RPM, p90 369, against median 307 and p90 400 for 225 hooded balls. With the hood within 1.5 deg of the table, backward balls fed -50..0 RPM from the table's speed scored 14 of 19, -100..-50 6 of 12, 0..+50 3 of 8 (3 long), +50..+100 2 of 5 (3 long).
- `VOLLEY_BACKWARD_FIRST_BALL_TRIM_DEG` (-3.7) / `VOLLEY_BACKWARD_LATER_BALL_TRIM_DEG` (-1.0): from Volley Tuning's B3 on 15 Nov, 30 volleys. First balls at -3.5..-4.0 went in 7 of 9 against 2 of 6 at -2.7..-3.0; later balls went in 4 of 4 at -1.5 but 20 of 34 at -0.2..-0.4, with 11 short.
- `VOLLEY_FIRST_BALL_TRIM_DEG` (-0.75) / `VOLLEY_LATER_BALL_TRIM_DEG` (-0.75): the medians of the eight forward spots confirmed by 15 Nov 1140; first-ball trims ranged -2.4..+0.9, later-ball trims -1..0.
- `VOLLEY_USE_TUNING_CSV` (true): turned back on 15 Nov, when spots 4-10 were confirmed at that day's full feed, 80-90% in. It was off from 12 Nov while the CSV held only the old ringing recovery and slow feed, whose spot 0 was still unconfirmed.
- `VOLLEY_FEED_POWER` (1.0): at 0.6 the feed took 0.21-0.24 s a ball, and feeder 0.6 with indexer 0.3 left the 4th ball stuck (12 Nov).
- `VOLLEY_INDEXER_SCALE` (1.0): at 0.3 the next ball took 2.9 s to come up (12 Nov 211957). Volley Tuning at 0.74 m (14 Nov 224053): full feeder and indexer went 8 of 8 in, every pulse a ball, the last out 1.2 s after the first feed; indexer at 0.3 (feeder 0.6) went 8 of 8 but a pulse in one volley came up empty and the four took 1.7-2.2 s; at 0.15 the four took 3.3-4.9 s.
- `VOLLEY_MIN_FEED_SPACING_S` (0.30): Volley Tuning's profile 0 fed 0.30-0.37 s apart and went 8 of 8 (14 Nov 224053). TeleOp's stack fed 0.12 and 0.20 s apart (230333), and the two balls out 0.12 s apart went into each other in the Cell.
- `VOLLEY_LEAD_BELOW_RPM` (150): replaying 13 Nov's gaps, the lead saved 0.18 s a ball (0.58 s each); the slowest tenth of balls leave ~70 RPM under target against ~35.
- `AUTO_VOLLEY_LEAD_BELOW_RPM` (0): with 150 the 4th preload left 0.17-0.20 s after the 3rd, wheels ~120 RPM under, and missed in every corner auto (15 Nov 153408, 154350); Volley Tuning's balls were ~0.34 s apart.
- `VOLLEY_HOLD_BETWEEN_BALLS` (true): was false (feeder stopped between balls).
- `VOLLEY_FAST_RECOVERY` (true) / `VOLLEY_OPEN_LOOP_BELOW_RPM` (600) / `VOLLEY_HANDBACK_RPM` (450): at 200/100 the wheels overshot to +600, dipped below 200 again and rang +-500 RPM for ~0.8 s every ball, which was most of the volley's time, and the swings read as double balls (12 Nov, 211957). Speed control alone recovering a 300-430 RPM dip to the band in ~0.25 s was seen in 9 Nov TeleOp.
- `VOLLEY_DOUBLE_DIP_RPM` (600): the 300-430 RPM single-ball drop under speed control was measured 5 Nov.

### VolleySettings (shooting.volley)
- Class Javadoc (an unconfirmed estimate can come from an old feed): spot 0's unconfirmed estimate is from 12 Nov's 0.6 feeder.

### VolleySpotDrive (shooting.volley)
- `EXTRA`: the positions are from TeleOp's hooded volleys in the 15 Nov matches: along the side wall 30-39 degrees off the Cell (6 balls, on spot 5's settings), and level with the Cell just outside the backward wedge (2 balls).
- `VOLLEY_ORDER`: hooded volleys at spot 0 went 86 of 260 IN (14 Nov).
- `BACKWARD_ORDER`: in the 15 Nov matches TeleOp's backward volleys left from near B3 and B4 at 29 and 28 degrees off the Cell.

### AimTrimSearch (tuning)
- `record` (step doubled from the third miss in a row on one side): halved to 0.25 deg, the step took 8 balls missing left in a row to move the aim 1.75 deg (7 Nov, 080156).

### BallDipDetector (tuning)
- `DIP_RPM` (200): the empty retry pulses dipping under 150 rpm were seen 6 Nov, 082517.

### ShotSearch (tuning)
- `record` (search reopens when most confirming balls miss on range): 7 Nov, 085204: the search called the range found at 3 of 10 in, its three confirming balls all went long, and two ins from an earlier session held the range at -2.0 deg.
- `plan` (`atLimit`, a limit off the quantum is reached within half of one): 7 Nov, 082948: with the hood limit at -3.93 deg, every ball fired at -3.9 went in and the long edge was never found, over 38 balls.

### SotmPath (tuning)
- `shape` (STRAIGHT and ARC fire time): the robot's 0.3-0.5 s lag getting going was seen 7 Nov, 100426.
- `trimmedToShot`: a line past the Cell at 1 m/s (7 Nov, 083925) had a shot for the first 0.25 s of its 0.7 s window and backed out of the table for the rest: three runs, no ball.

### SotmTuningSettings (tuning)
- `SPEED_PER_POWER_MPS` (1.25) / `POWER_AT_REST` (0.13): logs of 7 Nov: 0.47 m/s at 0.5 power, 0.9-1.1 m/s at full power, barely moving at 0.3.
- `TRACK_ALONG_GAIN_PER_S` (1.0): chasing a start lag of 0.13 m at 2.5/s surged a 0.3 m/s pass to 0.64 m/s (7 Nov, 100426).
- `TRACK_BRAKE_GAIN` (1.5): at 0.5 the stick never reversed; the plan braked from 1.0 to 0.2 m/s while the robot rolled on at 0.8 m/s into the fence check (7 Nov, 081827).
- `FIRE_CLOSE_MOVING` (true): a 0.5 m/s pass 0.55 m from the lip never fired in six runs (7 Nov, 092743).

## Subsystems and hardware

From `subsystems/`, `subsystems/hood/`, `hardware/` and `control/`.

### FeederSubsystem (subsystems)
- Feeder servo R reversed: a positive raw power running the tunnel backward on Mark 3 was found with Servo Check and driving, 27 Oct.

### FrameRecorder (subsystems)
- `FPS` (5): 10 fps at 640 wide slowed the robot loop from 67 to 88 ms (6 Nov).

### IntakeOverloadGuard (subsystems)
- `SEVERE_AMPS` / `LOAD_BASELINE_AMPS` block: the 3-4 A running and brief 5-7 A ball-handling bursts came from the 9 Nov loading logs.
- `INSTANT_RESET` (true): the drivers' call was made 13 Nov. That day's TeleOp ran the intake at a median 5-5.7 A (balls held against it), which spent the load budget in ~8 s, and three trips locked it out for the rest of the run (175517, lockout at 136 s).

### IntakeSubsystem (subsystems)
- `BEAM_BREAKS_WIRED` (false): the beam breaks have not been used since 13 Nov.
- `INTAKE_USES_ENCODER` (false): the intake has run in power mode since 27 Oct.
- `REVERSE_COAST_S` (0.3) / `REVERSE_RAMP_S` (0.2): were 0.75 + 0.75, so a tap of B never reversed the intake (13 Nov 191420); the quicker times, at the gearbox's risk, were accepted 14 Nov.

### MecanumDriveSubsystem (subsystems)
- `HEADING_KP` (1.0) / `HEADING_KD` (0.06): KP 3 with KD 0.18 swung 40-70 degrees either side of a still target at about 1 Hz, as the turning model predicts with the 60-120 ms delay (5 Nov, 093250). KP 1 (without the latency prediction) overshot by 10-20 degrees; judging the error HEADING_LATENCY_S ahead held a still target within 8 degrees throughout the simulation.
- `HEADING_KV` (0.16): strafing near the Cell swings the aimed heading by up to ~80 deg/s, and P alone trailed it by 10-20 degrees (5 Nov, 084358).
- `HEADING_KS` (0.16): 0.12 left the robot stuck 7-9 degrees off, outside the shot tolerance (5 Nov, 084358).
- `HEADING_KS_ABOVE_RAD` (1 degree): at 2.5 degrees the lock stalled 1.8-2.7 degrees short, always the same way, and standing shots at 1.3-1.6 m went in under half the time, missing to the right (5 Nov, 084758-091609).
- `HEADING_KS_BOOST_PER_S` (0.8): a least power of 0.16 left the robot 8.6 degrees off for 4.5 s, wheels at 0.16 and 12.2 V, where 0.15 had turned it the run before (5 Nov, 083302). At a boost rate of 0.2/s the last 8.6 degrees still took 1 s standing (5 Nov, 084933).
- `HEADING_KS_BOOST_MAX` (0.25): at 0.15 (0.31 in all) the robot sat 5.2 degrees off a 4 degree tolerance for over 2 s, wheels pushing, robot still, no shot (13 Nov 185305); 6.5 s stuck that day, 7.9 s on 8 Nov.
- `HEADING_STUCK_RATE_RAD_S` (15 deg/s): at 5 deg/s the robot crept the last 15 degrees at 10-15 deg/s on 0.16-0.23 power, never counted as stuck, and took 2.1 s to line up standing (5 Nov, 083228).
- `HEADING_MOVING_SCALE` (1.0) / `HEADING_MOVING_FULL_AT` (0.5): turning while strafing hard gave 10.6 rad/s per unit turn power, against 6.3 standing. With the standing gains the turn rate swung 0-2.4 rad/s around a 0.6-1.0 rad/s target and the aim was 10-20 degrees off (5 Nov, 081829). In the turning model, scaling and dropping the least power cut the error from 8.6 to 2.9 degrees rms. On the field at a scale of 0.6, strafing at full stick, the robot turned only 4-7 rad/s per unit (wheels saturated) and trailed the aim by a steady 8-10 degrees, just outside the 8-degree tolerance (5 Nov, 084933).
- `TWIST_FF_FORWARD` (-0.06) / `TWIST_FF_RIGHT` (0.15): per unit of robot-frame drive power, forward turned the robot CCW at +0.35 rad/s and strafing right turned it CW at -0.77, against 2.6 rad/s per unit of turn power while rolling (34k loops of 7 Nov passes; same within 10% for 0.05-0.25 s response models). Backing away-left at 0.45 m/s it turned CCW at 10-30 deg/s while the lock asked 0.14 CW, and sat 5-9 degrees off for the whole window: never fired, five runs (7 Nov, 080429). At 90% of the fit (-0.13, +0.30) the same pass sat 8 degrees off the other way (7 Nov, 082329).
- `KEEP_TURN_DIRECTION_ABOVE_RAD` (150 degrees): the robot wiggled in place for 0.7 s instead of turning (5 Nov, 092433).
- `STICK_MIN_POWER` (0.18): from the 4 Nov logs, wheels were mostly still below 0.25 power.
- `TURN_STICK_DEADBAND` (0.15): the untouched turn stick spun the robot at 60-115 deg/s while the driver went straight (5 Nov, 093520).
- `DRIVE_USES_ENCODERS` (false): DriveFL's encoder was seen counting backward on 27 Oct.
- Constructor motor directions: from the Mark 3 log of 27 Oct 17:51; the forward-stick spin and backward rotate were read on the Pinpoint pods.

### OdometrySubsystem (subsystems)
- Class Javadoc (pod ports): Mark 3 had the pods swapped on 27 Oct.
- `PINPOINT_VELOCITY_IS_FIELD_FRAME` (true): checked 4 Nov; facing 180, the field-frame velocity matched the pose change to 4 in/s, while the robot-frame velocity was 30 in/s out.
- `offsetXM` / `offsetYM` / `offsetHeadingRad`: writing a pose into the Pinpoint lost 1.1 degrees per write while turning, and the heading was 23 degrees out after 21 corrections (5 Nov, 081521).
- `FAULT_AFTER_S` (0.3): single bad reads cleared within 60 ms (5 Nov); the stuck FAULT_BAD_READ that let the field test push the robot off the field was 5 Nov, 085633.
- `RESET_NEAR_ZERO_M` (0.10) / `RESET_NEAR_ZERO_RAD` (10 degrees): in 223434 the first read after the reset was already 3 degrees turned.
- `getPose()`: a correction written from a NOT_READY read sent the heading from -172 to 0 (4 Nov, at 357 s).
- `readDevice()`: 13 Nov 223434, the reset from INIT finished 0.17 s after START, and the pose jumped 0.45 m and 32 degrees, so the Scoring Auto drove off the field.

### RampSubsystem (subsystems)
- `STOWED_US` (1490): the positions were found with Ramp Servo Check on 14 Nov, after the arm was refitted on the spline. Stowed was 1430 on 14 Nov, moved 60 us (~8 degrees) further back on 15 Nov.

### ShooterStallGuard (subsystems)
- Class Javadoc (burnout): measured on Mark 3 on 25 Oct. The 6.4-7 A at 480-700 ticks/s ran for about 3 minutes across several runs. Once one motor died, the other drew 7 to 9.6 A at 0 ticks/s. Running freely the same motors drew 0.3 to 0.5 A at 2300 to 2460 ticks/s.
- `STALL_VELOCITY` (750): 1200 was within normal running; a far shot at 3700 RPM (1180 ticks/s, 4-5 A) tripped the guard at 1200 (5 Nov, 082111).

### ShooterSubsystem (subsystems)
- `LEAD_S` (0.1): the lag was measured with Flywheel Tune, 5 Nov.
- `OPEN_LOOP_BELOW_RPM` (400) / `HANDBACK_RPM` (150): spinning up from 0 to 7281 RPM under speed control wound up the hub's integral, and the wheels then ran 100-340 RPM fast at a 3300 target for 20 s (7 Nov, 094905).
- `TARGET_RESEND_RPM` (20): re-sending the wobble every loop kept the wheels ~75 RPM low (5 Nov, 080236).
- `VELOCITY_P` (150): the SDK's 10 cut only ~2% power for 170 RPM too fast, so the wheels took 2-3 s to coast down to a lower target (5 Nov, 084358). At 30 they still took 1.6 s to fall 600 RPM and ran 145 RPM fast following a target falling 600 RPM/s while driving at the Cell; 150 took 0.4 s and held as steady; 250 began to oscillate (Flywheel Tune, 5 Nov).
- `VELOCITY_F` (12.9): at 13.8 the wheels settled at 4058 RPM for a 3780 target.

### VisionSubsystem (subsystems)
- `COMP_EXPOSURE_MS` (2) / `COMP_GAIN` (20): the venue was the 15 Nov competition. At the home 5 ms / gain 60, camera R's frames there averaged 181 of 255 with 22% clipped white and tag blacks lifted to ~90, and decision margins were R 77, L 41 at 1-1.5 m, against ~140 at home. 3 ms / gain 30 lets in about a third of the light of 5 ms / gain 60. At 3 ms / gain 30 (the 10:05-10:11 runs) camera L frames averaged 241-250 with 84-93% clipped and its near margins fell to 62; camera R 206-244 with 27-67% clipped. Then 2 ms / gain 15 was tried. At 2 ms / gain 15 (10:14-10:27): L margins 101-104 at 15-45% clipped, 119-129 at 45-75%; R 88-91 under 30% clipped, 115-123 over 45%.
- `VISION_THREAD_NICE` (8): at 0 the robot loop ran 77 ms; at 10 it ran 54 ms and the cameras lost under 1 FPS (7 Nov, 092018).
- `LIVE_VIEW` (false): the render thread's CPU use was seen 7 Nov.
- `LENS_LEFT` / `LENS_RIGHT`: the tools/camera-calibration results are from 1 Nov; the left camera's 1% focal length scale-up was fitted with the mount on 6 Nov.
- `RESTART_AFTER_NO_FRAMES_S` (2.0): camera R's USB error and 0 FPS match was on 4 Nov.
- `cameraWorkers`: the camera dropped off USB six times in 3 s and the loop froze with the flywheel on for 4 minutes (5 Nov, 080143).
- `WORKER_STUCK_S` (5.0): 13 Nov 165003, camera R's restart never returned and it gave no frames for the whole 109 s.
- `FrameClock`: the frozen stream kept reporting 9.2 FPS, and the robot drove 10 s blind (5 Nov, 082057).
- `frameTimesTrusted()` / `MIN_TRUSTED_FPS` (5.0): camera R's first frame after a restart (4.4 fps) was taken before a 36 degree turn but placed after it, which put the Cell 0.7 m off; TeleOp then aimed 0.34-0.69 m from the lip when it was nearer 1 m (15 Nov, 080649).
- `detach()`: Android killed the app at 1 GB (6 Nov, 10:12).

### HoodController (subsystems/hood)
- Class Javadoc, moving target: without the rate the hood waited for the target to get 0.2 degrees away and swung 1-1.5 degrees either side of it every 0.12 s (5 Nov, 094733).
- Class Javadoc, RUN_TO_POSITION: the hub's speed loop swung 3-6 degrees around the target (Hood Tune sweeps, 5 Nov).

### HoodEncoderGuard (subsystems/hood)
- Class Javadoc, belt skip: TeleOp 4 Nov, the motor reached full lift while the REV stopped 9 degrees short.
- Class Javadoc, REV not following: in 085313 three rebases to a dead REV drove the hood past its top.

### HoodFollowCheck (subsystems/hood)
- `WINDOW_MOTOR_TICKS` (60) / `SHARE` (0.25): sized on the logs of 5 Nov. The run that drove the hood past its top was 085313; the other real failure was 081558, where the REV stood still while the motor ran 260 ticks.

### HoodMotorTargeting (subsystems/hood)
- `motorTargetFor(..., motorMoving)`: switching offsets mid-move sent the motor 1.4 degrees back the other way, and the back-and-forth ended up to 1.9 degrees off (Hood Tune, 5 Nov).

### HoodSubsystem (subsystems/hood)
- `FAST_PERIOD_MS` (8): at 5 ms the main loop ran 65 ms against 40 without the fast loop (7 Nov, 092936).
- `FAST_KS` .. `FAST_KG`: the open-loop pulse fit was done with Hood Tune CHARACTERISE on 5 Nov.
- `FAST_KP` (0.12): 0.2 hunted 2-3 degrees either side of 38-41 at 12 Hz, powered throughout, for seconds at a time (6 Nov: G8 shots, and 2 of 10 Hood Tune holds). At 0.12: no hunting in 10 holds, settling in 0.25 s mean, overshoot under 0.8 degrees.
- `FAST_DEADBAND_DEG` (0.3): a deadband / re-engage of 0.1 / 0.2 hunted, with 0.2-degree corrections, and the hood swung 31-34 at 8 Hz (5 Nov, 084004).
- `STEADY_S` (0.1): the gate passed with the hood swinging 2.5 degrees either side of its goal at 12 Hz (6 Nov, G8).
- `FAST_TRACK_RATE` (true): off is the 5 Nov behaviour.
- `HOME_OVERTRAVEL_TICKS` (100): on 5 Nov (093943) homing ran 1785 ticks, over 3x the travel, for the full timeout without stalling.
- `HOME_SETTLE_MAX_S` (0.5): zeroed while still coasting, the zero sat 820 REV ticks (3.6 degrees) off the stop (093943).
- `fastLoopStoppedItself()`: Volley Tuning stalled 0.59 s on Circle (15 Nov, 074849), and the hood stayed at 56.7 of 74.2 degrees.

### HeadingHold (control)
- Class Javadoc: open loop the base swung up to 10-15 deg/s on its own while strafing at full stick (5 Nov, 093520).
- `CAPTURE_BELOW_RATE_RAD_S` (15 deg/s) / `CAPTURE_AFTER_S` (0.5): the released turn's 0.3 s coast went from 100 deg/s to 0 (5 Nov, 093520).

### LoopWatchdog (control)
- Class Javadoc: the loop stopped in INIT with the right camera opening twice on 6 Nov (091421, 093333).

### MotorReversalGuard (control)
- `update()` reversal from a stopped intake: the rule that time already stopped counts as coasting dates from 14 Nov.

### HardwareNames (hardware)
- `BEAM_BREAK_TOP` / `BEAM_BREAK_BOTTOM`: unused since 13 Nov.

### PinpointSettings (hardware)
- `X_OFFSET_MM` (-71.5) / `Y_OFFSET_MM` (0.1): the old offsets (0, -105) made spinning in place read as 0.07 m/s forward and 0.10 m/s left per rad/s (fits 0.85-0.99 over four runs), which worked back to offsets of about (-65, 0) (5 Nov, 084206).
- `X_DIRECTION` (REVERSED): TeleOp log 3 Nov 22:22, wheels driving the robot backward moved the pose forward in 30 of 31 samples.
- `Y_DIRECTION` (REVERSED): pushing the robot right raised Y (4 Nov).

## Robot, field and vision

From `robot/`, `field/` and `vision/`.

### AimSelector (robot)
- `aimAtCell` (shot direction picked once a press): picking only a ready backward shot turned the robot 180 degrees to the hooded one while the driver was still moving at the press (5 Nov, 092433).
- `aimAtCell` (no vision while spinning fast): the spinning frame's pose moved the Cell 0.17 m during one shot (5 Nov).
- `aimAtCell` (aim at the up Cell's field position): the tracker's Cell, last seen 6-20 s before, had been dragged 0.38 m toward the down Cell, and the balls went into the down Cell (5 Nov, 081558).
- `aimAtCell` (pick whichever shot is ready soonest): added 8 Nov.
- `aimAtCell` (preposition switch held for PREPOSITION_SETTLE_S): the belt skipped while the hood swung through its whole travel (4 Nov).

### AlliancePicker (robot)
- `allianceWarning`: a Shoot Auto set to BLUE saw only RED's Hive tags and never fired (13 Nov, 193125).

### BallPath (robot)
- `update` (intake ease clears on a real LT re-press, last roller stays stopped): seen 14 Nov 222230.
- `update` (volley indexer at full power between balls): without it the 4th ball took 2.9 s to come up (12 Nov).
- `update` (the auto feeds at full power): its slowed last roller brought the next ball up too little (14 Nov 220201).
- `feederCommand` (feeder held back whenever nothing shoots): balls left the stack between shots in Lebot Teleop on 12 Nov; the auto lost balls on the way to the Flower on 14 Nov 211610.
- `loadingLatchesClear` (LT re-press clears only with the flywheels stopped): 14 Nov 222230, a press 5.7 s after a packed stack was let go cleared it with the wheels at 3500 RPM, and the front ball was shot out.
- `intakeCommand` (auto stops the intake on a full stack instead of ejecting): 13 Nov 224556 ejected at the rear Flower for 3.2 s.
- `autoStage` (auto keeps the last roller staging): 14 Nov 220201 the preload shot 2 of 4, the Garden and Flower volleys none, with the last roller still for 2 s before each.

### BatteryMonitor (robot)
- `swapBattery`: two 9 Nov runs that started at 11.1-11.9 V fell to 6.8-6.9 V, and every auto sags to 9.6 V at spin-up even from 12.6 V.

### DriveControl (robot)
- `driveAuto` (heading lock does the auto's final turn): Pedro's turn stalled 8 degrees off at 0.14 power (14 Nov).

### DriverConstants (robot)
- `TELEMETRY_DETAIL` (false): before 14 Nov the TeleOp Driver Station always showed every subsystem's state.
- `INTAKE_ONLY_ON_LT` (true): introduced 13 Nov; before that the intake was always on (what false still gives).

### FireControl (robot)
- `checkReady` (`shotLost` stops a feed under way): a ball left 3 degrees of hood or 37 degrees of heading off when the driver set off or turned back mid-feed (5 Nov, 095605).
- `updateFeed` (held fire counts as released for the volley gate): the first ball got the last 0.06 s of a pulse and fell short (6 Nov, 081518).

### Robot (robot)
- Volley mode field comment: volley mode is used by Lebot Teleop and Volley Tuning (also in the `// ---- Volley (Lebot Teleop RB, Volley Tuning) ----` section header).
- `autonomous` (Scoring Auto fields): added 6 Nov on feature/auto.
- `init(boolean stillInInit)`: Lebot Teleop's INIT drove the hood down from the auto's ~61 degrees, which G403 bans between AUTO and TELEOP.
- `refreshDriver`: merging lets either gamepad drive; off with Main.PANELS_ENABLED (both visible in the code).
- `initLoop`: the driver picks the alliance here (alliancePicker.pollAlliance(), visible in the code).
- `start`: sets the heading from `placedHeadingRad` (visible in the code).
- `seedPose`: poseValid stays false until the first tag fix (the method sets only poseSeeded).
- `driverForwardHeadingRad`: the sticks, the start heading and A all use it.
- `periodic` (shooter set before the ready check): a hood target that had just moved 3 degrees was judged against the old one and the ball missed (5 Nov, 095605).
- `fireTrigger`: 13 Nov 224556, with only the aim taking autoShoot, shotRequested read the blank gamepad's RT, the indexer never staged or fed, and the auto shot nothing in 29 s.
- `startFromAuto`: the pose counts as placed, so field-centric driving and the aim are right from the first loop (as `startAt`'s Javadoc says).

### RobotTelemetry (robot)
- `fieldSender` (field sent from its own thread): 5 Nov the OpMode thread hung in field.update(), the robot stopped responding and the app restarted from "stuck in stop()".
- `due` (Driver Station telemetry at 4 Hz): formatting cost 10-18 ms a loop (7 Nov, 080123).
- `addPowerTelemetry`: the intake drew 5-6 A for most of a match (13 Nov).

### ShooterControl (robot)
- `update` (shot gone for a moment keeps its speed, PRESPIN_HOLD_S): NO_SAFE_SHOT flickers lasted a median 0.42 s, p90 1.2 s; dropping to IDLE braked the wheels a median 1350 RPM, 478 times that day (13 Nov).
- `update` (wheels and hood follow the shot for now, not the one at release): holding the release shot put them a feed delay early, 4 degrees of hood at 20 degrees/s (5 Nov, 080236).
- `update` (prespin at the shot's speed): spin-up of 0.8 s from idle measured 4 Nov.
- `update` (hood held just above the lower stop): with target 0 the hood stayed ~36 degrees up.

### CellGeometry (field)
- `CELL_REF_UP_UP_IN` and the other Cell reference offsets: measured against the SDK on 4 Nov.

### FieldGeometry (field)
- `ALLIANCE_FORWARD_RED_RAD` (Math.PI / 2): driver positions confirmed by the drivers on 6 Nov.

### FieldRoute (field)
- `FIELD_LIMIT_M` (1.35): at 0.15 m inside the perimeter, a pose that went 0.4 m wrong took the robot off the mat (5 Nov, 081516).
- `RELOAD_STICK` (0.4): into the corner at GOTO_STICK the robot was still at 0.6 m/s 0.15 m out, and its stop fell 4 cm past the check (7 Nov, 080429).
- `RELOAD_SLOW_M` (0.5): the whole 2.3 m leg at RELOAD_STICK crept at 0.1 m/s (7 Nov, 085204).
- `GOTO_MIN_STICK` (0.35): 0.18 shaped to 0.19 power and stalled 9 cm short.
- `GOTO_STUCK_BOOST_PER_S` (0.3): 0.35 sideways (0.26 power) sat 11 cm short until skipped (5 Nov, 082021).
- `GOTO_BLIND_S` (3.0): driving to the blue side the camera saw no tags for 20 s while the pose barely moved (5 Nov, 080742).
- `CROSS_GAIN` (2.0): aimed only at the end, 2.5 m down the edge drifted 0.21 m out into the fence (5 Nov, 082500).
- `goToStick` (boost lifts a cap below GOTO_STICK): at RELOAD_STICK (0.29 power) the robot sat still for 5 s at a time on the way to the reload corner and had to be pulled (7 Nov, 082948).

### PoseHandoff (field)
- Class Javadoc: after the Scoring Auto ended at -129 degrees, TeleOp started at +90 and drove 140 degrees rotated until its first two-tag fix, 4.7 s in (13 Nov 224556 -> 225324). The first fix took 2.9 s typically, and 26-118 s at worst.

### CameraMounts (vision)
- `LEFT_CAMERA`: yaw and pitch fitted on the 6 Nov calibration drive, from 824 sightings both cameras made of the same tags.

### ClusterFromTags (vision)
- Class Javadoc (Cell reference offset): measured against the SDK cluster detector on 4 Nov.

### VisionConstants (vision)
- `MAX_SPEED_FOR_VISION_MPS` (0.6): odometry held through the field test's moves (fixes of 1 mm still), while frames taken above 0.6 m/s or 35 deg/s were off by 5 cm and 2 degrees (median) and up to 13 cm and 11.5 degrees (5 Nov field tests, 7381 fixes).
- `MAX_SPEED_FOR_VISION_HEADING_MPS` (0.3): the odometry's own heading turned 158 degrees on the spot without a degree of error.
- `MAX_FRAME_AGE_S` (0.5): the 0.3 s average detector lag was measured 4 Nov.
- `POSE_OVERRIDE_FRAMES` (5): 22 rejected fixes agreed within 5 cm on a 2 m error (5 Nov, 082826).
- `MIN_FIRST_HEADING_PAIRS` (3): one pair put the start heading 28 degrees out, while 9 to 13 pairs a few frames later agreed on the truth (6 Nov, 080409).
- `HEADING_OVERRIDE_FRAMES` (5): a bad Pinpoint read left the heading 172 degrees out (4 Nov).

### VisionCorrector (vision)
- `upCellAimPoint`: Cell sightings agreed with it within 4 cm (5 Nov, 081558).
- `headingSeeded`: from its start spot the Shoot Auto saw one tag and got no fix in 30 s on 5 runs (13 Nov), firing 24-28 balls on the placement alone. Heading fixes come from pairs as before this was added.
- `process` (spread check after turning by the pairs' heading): standing still, five tags agreed the heading was 13 degrees out and every frame was refused for spread (5 Nov, 080149).
- `process` (NO_HEADING): one tag with the start heading 154 degrees out put the robot 2 m away, on the far side of the Cell, for the whole run (5 Nov, 082826).

## Autos and OpModes

From `auto/`, `opmodes/` and `opmodes/check/`.

### AutoField (auto)
- `START`: until 14 Nov 213317 the robot was put in the loading zone (illegal), and the camera's late correction made the preloads shoot 0.6 m short of the spot.
- `GARDEN_PUSH_M` (0.10): odometry's drift leaving the sweep short of the corner's last Pollen, aimed 2 cm short, was seen 14 Nov 214945.
- `FLOWER_STANDOFF_M` (0.45): 0.25 m was too close (14 Nov run).
- `GARDEN_EXIT_CONTROL` ({1.40, -0.95, 0}): on 14 Nov the straight drive passed the Flower 1.45 m out from the middle and hit it. Curved, it passes 1.21 m out or more.
- `PARK_WALL_GAP_M` (0.10): the rule that the robot may not touch the wall on PARK was learnt 15 Nov.
- `VOLLEY_SPOT` ({-1.00, -1.05}): its volleys there went 46 of 52 in, 0.74 m from the lip, no double balls (14 Nov 224053). The old spot, 1.0 m and 20 degrees off, now and then lost a ball outward.
- `AIM_INWARD_M` (0.0): 0.03 made up for outward misses from the old spot.

### AutoSequence (auto)
- Class Javadoc: the step plan is the one of 14 Nov.
- `SECOND_FLOWER` (false): turned off 14 Nov.
- `EMPTY_S` (2.0): at 0.6 s it gave up after one empty pulse, 14 Nov 220630: the Flower volley quit after 1.1 s of feeding with its balls still coming up.
- `VOLLEY_WAIT_S` (4.0): 3 s cut the preloads off after 2 balls on 14 Nov 212840: the first left 2 s after stopping, and the 2 left in blocked the Garden's intake.
- `GARDEN_SWEEP_MAX_S` (3.5): 0.43 m took 1.9 s on 14 Nov.
- `FLOWER_INTAKE_S` (3.0): 14 Nov the drive in took 1.2 s, then it sat 2.9 s in the Flower, empty.
- `WIGGLE_RAD` (7 degrees): 12 degrees was too much (14 Nov).
- `AGITATE_RUN_S` / `AGITATE_STOP_S` (0.4 / 0.2): the agitation was added 15 Nov for balls jammed in the intake's mouth.
- `BACK_OUT_MAX_S` (2.0): 0.28 m took 0.8 s on 14 Nov.
- `intake()`, `SHOOT_PRELOAD`: the full stack pushed a ball through the held feeder on the way, with the wheels spinning (14 Nov 220630).
- `intake()`, `SHOOT_GARDEN` / `SHOOT_WALL_FLOWER`: 14 Nov 214244 the Garden's balls sat in the intake's mouth, the intake switched off as the drive began, and the volley shot none; 214945, with it off once stopped, every volley left its last balls in.

### BaseAuto (auto)
- `FACE_FLOWER_M` (0.10): 14 Nov Pedro's hold stalled 6 cm off the Flower stop for 2 s.
- `onPsiKitInit`, `setVolleyMode(true)`: with volley mode off, each ball was a one-ball shot and the last of every volley stayed in (14 Nov 214945).
- `onPsiKitStart`, `setDriveBrake(true)`: Pedro's constants were measured on BRAKE on 13 Nov.
- `pedroPowers`, waiting for odometry: 13 Nov the Pinpoint calibrated for 0.2 s after START on both runs.

### CornerSequence (auto)
- Class Javadoc: the corner auto was made 15 Nov to fit around a partner whose auto takes the Garden side.
- `WAIT_S` (5.0): set 15 Nov.
- `JIGGLE_RUN_S` / `JIGGLE_REVERSE_S` (0.4 / 0.15): 15 Nov the volley after the Flower fed 2 s and shot none.
- `EDGE_GAP_M` (0.10): 5 cm was too close (15 Nov).
- `CORNER_PARK`: 3 cm inside the zone's line went a little too far (15 Nov).
- `nearCell` (SCORING) and `gardenCell` (AUDIENCE): chosen 15 Nov.
- `preloadSpot`: spot 5 went 86-88% in on 14-15 Nov. The earlier spot, 0.2 m out of the corner and 1.37 m from the lip, was inaccurate.
- `intake()`, `SHOOT_PRELOAD`: same evidence as AutoSequence's, 14 Nov 220630.
- `intake()`, `SHOOT_GARDEN_SIDE`: the Flower's balls sitting in the intake's mouth was seen 14 Nov.

### ScoringAuto (auto)
- `WALL_PUSH` (0.25): 14 Nov Pedro alone stalled 5.8 cm off the wall at 0.14 strafe, and swept 2.5-3 cm off.
- `stepArrived`, Flower stall: 14 Nov 212840 the drive into the Flower stopped 12 cm short.

### ShootAuto (auto)
- `DRIVE_TIMEOUT_S` (1.5): the odometry faulted (FAULT_BAD_READ) on 13 Nov 191050 and 191103 and the pose never moved; the old 3 s let it drive blind.
- `DRIVE_MAX_STEP_M` (0.4): at 0.12 it also dropped real steps, so it drove 0.86-1.48 m for 0.6 (13 Nov). The one real jump seen was 2.0 m.
- `SEARCH_TURN_S` (0.3): the steady search turn seeing nothing was 13 Nov, 193125.
- `PARK_MAX_SPEED_MPS` (1.6): at full stick it ran 0.45 m past the zone, swung back past it again and parked 0.19 m off (13 Nov 200130 and 215133). The 1.6 m/s top speed is from the 13 Nov logs.
- `PARK_VEL_GAIN` (1.0): with stick 0.2 the turned robot sat 0.15 m short for 5 s (13 Nov 193927).
- `PARK_DONE_MPS` (0.15): 220620 was "there" at 0.43 m/s along the wall, and the push carried it 0.08 m on, over the zone's edge.
- `PARK_PUSH_POWER` (0.5): 0.2 and 0.35 didn't move the robot into the wall (13 Nov).
- `onPsiKitLoop`, DRIVE phase: 13 Nov 191356 the tags first placed the robot from (0, 0) to (-1.46, 1.38) on one loop, which counted as the whole drive.
- `onPsiKitLoop`, SHOOT phase: the wait holding the turn, so every volley started a second late, was seen 13 Nov over 5 red runs.

### Main (opmodes)
- `LOG_MOTOR_CURRENT` (false): turned off for loop speed on 4 Nov.
- `NON_BULK_READ_PERIOD_S` (2.0): reading them every loop made the motor logging 62 ms of a 138 ms loop (4 Nov).

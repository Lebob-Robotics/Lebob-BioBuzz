# Flywheel mass tool

Lebob Robotics, FTC 29550 · BIOBUZZ 2026/27 · v1.0, 2 Oct 2026

Four Pollen in a row each take energy out of the flywheel, and the shooter
has to stay within ±50 RPM through all of them. This tool finds the
lightest wheel that does that, and how many grams to bolt on to the wheels
already on the shaft.

## Files

- `flywheel.js`: the physics, the mass sweep and the recommendation. The
  only copy. `node tools/flywheel/flywheel.js` runs the reference checks,
  then prints a recommendation for each shot case in `CASES`.
  `node tools/flywheel/flywheel.js hood` also runs the hood sweep (about a
  minute) and feeds its worst shot in as a case.
  `node tools/flywheel/flywheel.js gear` sweeps the second wheel's gear ratio
  across every case first.
- `inputs.json`: the shooter, motor and ball numbers. Most are estimates or
  guesses, see the table below.
- `test.js`: the reference results from the handoff as assertions. The CLI
  refuses to print if any fail.
- `flywheel.html`: the same, with plots. Open it, press Compute, move the
  mass slider.

## What it models

The flywheel is one rigid inertia on a linear DC motor curve. Both wheels
are lumped at the main wheel's radius, I = k m R². Each ball leaves at the
mean wheel speed over the contact, and takes its kinetic energy (plus spin,
divided by the transfer efficiency) out of the wheel. Between shots the
hub recovers in one of two ways:

- `full`: coasts for the reaction delay, then full power up the motor curve
  until it hits the target. A well-tuned or bang-bang loop.
- `ff`: holds whatever duty F gives at the target, so the gap closes at the
  motor's own rate. This is the current gains, where F dominates and P is
  weak.

Every shot is also fired with a ball 20% heavier and 20% lighter. The worst
exit-speed error across the four shots, against a nominal ball from a fully
spun-up wheel, has to sit inside the RPM tolerance as a percentage of the
target. The sweep runs 20 g to 2 kg in 10 g steps and takes the first mass
that passes, floored at the existing wheels, then adds 15% and rounds up to
5 g.

Spin-up above `maxSpinupS` is flagged. In `ff` the fix is a higher velocity
P gain; in `full` it is gearing up, spacing the shots, or a lower target.

Not modelled: friction, belt slip, battery sag during the burst, PID
overshoot, hood scrape (drop `transferEfficiency` to about 0.50 if the hood
presses hard), the second wheel's own inertia, and the target moving between
balls while the robot drives.

## Reading the output

    Hood worst case, backing away 2.2 m/s at 1.45 m (7.06 m/s): 4202 rpm (70% of 6000 free)
      passes from 140 g, spin-up 0.16 s, worst error 1.1% (allowed 1.19%)
      build 165 g: inertia 62 kg mm2, existing 39, add 59 g at 20 mm or 45 g at 22.9 mm

Inertia is what matters, so the added mass depends on where it goes. The
two radii are `addRadiusM` and 0.9 R near the rim. Spread it evenly and
balance the wheel afterwards. "Motor can't reach that speed" means the
target is above 97% of the drive's free speed at any mass; gear up or lower
the exit speed.

In `ff` mode the error curve is not monotone, it can pass at very low mass
(fast recovery) and again at high mass (small drop), so the CLI lists every
passing range.

## Gearing the second wheel

Both motors get one `setVelocity`, so the 24.5 mm wheel's surface turns at
less than half the speed of the 50.8 mm wheel. The ball can only leave at
the average of the two, so half the main wheel's speed is thrown away and
the difference puts about 780 rpm of spin on the ball. A gear or belt stage
between the second motor and its wheel fixes that without a code change.
`wheel2Gear` is the second wheel's rpm over the main wheel's.
`exitSpeedPerRpm` is taken as measured on the as-built 1:1 shooter, and the
slip fraction (kpr over the no-slip ceiling) is held constant, so kpr rises
with the ceiling when the second wheel is geared.

The sweep shows two effects pulling against each other. Gearing up lowers
the rpm a shot needs, which is what brings the fast fixed-angle shots back
within the motor's reach. It also lowers the wheel's stored energy at that
shot, so each ball takes a bigger bite and the passing mass creeps up, from
140 g at 1:1 to 160 g at 1.85:1 for the hood worst case. The small wheel's
own speed is the hard limit, `wheel2MaxRpm` (9000, a guess until the hub
and wheel are rated). With the defaults the knee is 1.9:1: the 10.178 m/s
shot goes from unreachable to 230 g, spin falls to almost nothing, and the
small wheel sits at 8900 rpm. Above 1.95:1 the fast shots hit the wheel
limit instead. The 11.27 and 12.535 m/s table rows never pass at any ratio,
so the fixed-angle layout cannot cover them with these motors.

The knee is sharp because the shot interval is a guess. At 1.85:1 the 230 g
pass is a single point, the mass where recovery time equals the 0.12 s
interval; lighter wheels recover fully and heavier ones fall further behind
each ball. The tool marks such passes as a knife edge and the gear table
only counts ranges at least 30 g wide. With 0.15 s between balls the same
230 g is a solid pass at 1.85:1, so measure the interval before choosing a
ratio.

If the hood goes ahead, 1:1 is fine, the hood cases pass at 140 g and 120 g.
Matched surface speed is 2.07:1 with these wheels and removes the spin
entirely; a second 50.8 mm wheel does the same thing with no gear stage.

## The exit speed per RPM problem

`tools/shots/inputs.json` has `exitSpeedPerRpm` at 0.0023. Two wheels at one
RPM cannot push a ball faster than their average surface speed, which for
50.8 mm and 24.5 mm wheels is 0.00197 m/s per RPM, so 0.0023 is 117% of
what the hardware can do and every RPM in `ShotTable.java` is too low. This
tool uses 0.00168 (85% of the ceiling) until it is measured. The fixed-angle
cases below were generated with 0.0023, but RPM × kpr is the physical speed
the trajectory needs, so those speeds stay valid when kpr is corrected.

## Shot cases

| Case | Exit speed (m/s) | Origin |
| --- | --- | --- |
| Hood worst case, backing away 2.2 m/s at 1.45 m | 7.06 | Hood sweep, 54° |
| Hood standing, 1.45 m | 5.71 | Hood sweep, 59° |
| Fixed standing, 1.45 m | 7.734 | ShotTable 3362.5 rpm |
| Fixed median table shot | 6.756 | ShotTable 2937.5 rpm |
| Fixed backing away 0.7 m/s | 10.178 | ShotTable 4425 rpm |
| Fixed backing away 0.9 m/s | 11.27 | ShotTable 4900 rpm |
| Fixed table maximum, backing away 1.7 m/s | 12.535 | ShotTable 5450 rpm |

## Measurements

Replace the estimates in `inputs.json` in this order, then rerun. The
result is not trustworthy until the first three are measured.

| Input | How to measure | Value | Date | Who |
| --- | --- | --- | --- | --- |
| `exitSpeedPerRpm` | Slow-motion video past a metre rule, as in `tools/shots/README.md` | | | |
| `shotIntervalS` | Time between balls leaving during a four-ball burst, from video or the shot log | | | |
| `existingWheelMassKg` | Weigh both wheels, hubs and shafts | | | |
| `controller`, `controllerDelayS` | `left_rpm` and `right_rpm` in the shot log during a burst: does the hub push full power after each ball (`full`) or drift back (`ff`)? | | | |
| `transferEfficiency`, `energySpreadPct` | Exit speed spread across a bag of Pollen at one RPM | | | |
| `addRadiusM` | Radius where weight can actually be bolted on | | | |
| `wheel2MaxRpm` | Bearing and wheel rating for the small roller at speed | | | |

## Follow-ups

- Set `exitSpeedPerRpm` in `tools/shots/inputs.json` to the measured value
  and regenerate `ShotTable.java`.
- Raise the velocity P gain so the hub pushes full power after each shot,
  then confirm from the log.

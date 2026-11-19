package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.shooting.feed.FourthBall;
import org.firstinspires.ftc.teamcode.shooting.feed.IndexerRetry;
import org.firstinspires.ftc.teamcode.shooting.feed.StackPressure;
import org.firstinspires.ftc.teamcode.shooting.feed.WheelContact;
import org.firstinspires.ftc.teamcode.shooting.volley.VolleyConstants;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem.FeederState;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem.IndexerState;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.IntakeState;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.ShooterState;
import org.psilynx.psikit.core.Logger;

/** The Robot's intake, indexer and feeder decisions each loop, from the shot, LT, B and how full the stack is. */
final class BallPath {
  private final Robot robot;

  private final IndexerRetry indexerRetry = new IndexerRetry();
  boolean intakeOn = true;
  private boolean wasFull = false;
  private final FourthBall fourthBall = new FourthBall();
  private final WheelContact wheelContact = new WheelContact();
  private boolean wheelContactFull = false; // A loading ball touched the wheels: the stack is full
  private final StackPressure stackPressure = new StackPressure();
  private boolean stackPacked = false; // The intake pushed a packed stack: it eases off and the last roller stops
  private boolean intakeEased = false; // The intake at PACKED_INTAKE_POWER; clears sooner than stackPacked
  private boolean wasLtPressed = false;
  private double ltReleasedAtS = Double.NEGATIVE_INFINITY;

  BallPath(Robot robot) {
    this.robot = robot;
  }

  /** True while the stack is full: a 4th ball in the beam, or a loading ball touched the wheels. */
  boolean stackFull() { return wasFull; }

  /**
   * Intake collects while LT is held (always, with INTAKE_ONLY_ON_LT off) and reverses at full capacity. An aimed
   * shot holds the feeder backward and stops the last roller until READY. LT advances that roller gently against
   * the held feeder until a loading ball touches the spinning flywheels (WheelContact), the sign of a full stack.
   * Between shot feeds, the upstream stage keeps the second/third balls moving toward the stopped last roller.
   */
  void update(boolean aiming, boolean ready) {
    FireControl fire = robot.fireControl;
    boolean shooting = aiming || (robot.driver.y && robot.buttonsFree());
    boolean fireRequested = robot.shotRequested();
    boolean shotAllowed = fireRequested && !robot.holdFire;
    boolean enabledShot = shotAllowed && !fire.loadingShooter.blocksSpin();
    boolean ejecting = robot.driver.b && robot.buttonsFree();
    double feederPower = robot.volleyMode && fire.volleyShot && enabledShot ? robot.volleyFeedPower() : 1.0;
    double loadTrigger = Robot.loadTrigger(robot.driver.left_trigger, robot.autoIntake);
    boolean ltPressed = loadTrigger > Robot.TRIGGER_PRESSED;
    boolean loading = ltPressed && !shotAllowed;
    // Full until a ball leaves, B ejects or LT is pressed again after a real let-go (the way out if a bump looked
    // like a touch). A tap doesn't count, or tapping LT pushed the packed stack at full power on every press
    double nowS = robot.clock.seconds();
    boolean wheelsSpinning = Math.abs(robot.shooter.getMaxMeasuredRpm()) > WHEELS_STOPPED_RPM;
    if (loadingLatchesClear(ltPressed && !wasLtPressed, nowS - ltReleasedAtS, ejecting, fire.feedGate.ballLeft(),
        wheelsSpinning)) {
      wheelContactFull = false;
      stackPacked = false;
      intakeEased = false;
    }
    // The intake's ease clears on a real LT re-press even with the wheels spinning: the last roller stays stopped
    // (stackPacked) until a ball leaves, so the front ball still can't creep into the wheels
    if (intakeEaseClears(ltPressed && !wasLtPressed, nowS - ltReleasedAtS,
        stackPressure.runningFree(nowS, intakeEased && robot.intake.getIntakeState() == IntakeState.INTAKE,
            robot.intake.getAmps()))) {
      intakeEased = false;
    }
    if (!ltPressed && wasLtPressed) {
      ltReleasedAtS = nowS;
    }
    wasLtPressed = ltPressed;
    boolean watchingWheels = loading && !wheelContactFull && !robot.tuningMode
        && robot.indexer.getIndexerState() == IndexerState.LOAD && robot.shooter.getShooterState() == ShooterState.AIM;
    if (wheelContact.update(robot.clock.seconds(), watchingWheels, robot.shooter.getRpm(),
        robot.shooter.getTargetRpm())) {
      wheelContactFull = true;
    }
    Logger.recordOutput("Intake/WheelContactFull", wheelContactFull);
    // Not in the auto, whose intake pushes into the Flower's balls the whole time and has no LT to start over.
    // Still packed (last roller stopped), a full intake pushing hard again eases again
    boolean watchingIntake = StackPressure.ON && loading && !intakeEased && !robot.autoIntake && !robot.tuningMode
        && (robot.indexer.getIndexerState() == IndexerState.LOAD || stackPacked)
        && robot.intake.getIntakeState() == IntakeState.INTAKE;
    if (stackPressure.update(robot.clock.seconds(), watchingIntake, robot.intake.getAmps())) {
      stackPacked = true;
      intakeEased = true;
    }
    Logger.recordOutput("Intake/StackPacked", stackPacked);
    Logger.recordOutput("Intake/Eased", intakeEased);
    robot.intake.setIntakePower(intakeEased ? StackPressure.PACKED_INTAKE_POWER : 1.0);
    boolean fourBalls = fourthBall.update(nowS, robot.intake.isBeamBlocked(), fire.feedGate.ballLeft(), ejecting,
        loading && robot.indexer.getIndexerState() == IndexerState.LOAD);
    Logger.recordOutput("Intake/BeamBlocked", robot.intake.isBeamBlocked());
    Logger.recordOutput("Intake/FourBalls", fourBalls);
    Logger.recordOutput("Intake/BeamBlockedLoadingS", fourthBall.blockedLoadingS());
    boolean full = fourBalls || wheelContactFull;
    Logger.recordOutput("Intake/Full", full);
    if (full && !wasFull) {
      robot.physicalDriver.rumble(300);
    }
    wasFull = full;

    boolean intakeWanted = robot.tuningMode || !DriverConstants.INTAKE_ONLY_ON_LT
        || loadTrigger > Robot.TRIGGER_PRESSED;
    intakeOn = intakeWanted;
    robot.intake.setIntakeState(intakeCommand(full, ejecting, intakeWanted));
    boolean feeding = enabledShot && (!aiming || ready);
    // A volley slows the indexer only while a ball feeds, so the next can't follow it in; between balls it stages
    // the stack at full power against its stopped last roller, or the 4th ball is slow to come up
    // The auto feeds at full power: its slowed last roller brought the next ball up too little
    double indexerPower = robot.volleyMode && fire.volleyShot && feeding && !robot.autonomous
        ? robot.volleyIndexerPower() : 1.0;
    IndexerState indexerCommand = indexerCommand(ejecting, fireRequested, feeding, loadTrigger,
        robot.holdFire || fire.loadingShooter.blocksSpin(), full);
    indexerCommand = packedLoad(indexerCommand, stackPacked);
    indexerCommand = autoStage(indexerCommand, robot.autonomous);
    indexerCommand = agitated(indexerCommand, robot.autonomous && robot.autoIndexerPaused, robot.autoIndexerReverse);
    if (!ejecting && (fire.loadingShooter.clearing() || (loading && !fire.loadingShooter.canLoad())
        || (robot.spinOnlyOnFire && enabledShot && !feeding && fire.feedGate.feedStarts() == 0))) {
      indexerCommand = IndexerState.STOP;
    }
    // Retry only after a volley has actually started. All release, clearance and shot gates still apply.
    boolean retryAllowed = fire.volleyShot && enabledShot && !ejecting && fire.feedGate.feedStarts() > 0
        && (indexerCommand == IndexerState.STAGE || indexerCommand == IndexerState.FEED
            || (robot.autonomous && indexerCommand == IndexerState.LOAD));
    indexerRetry.update(robot.clock.seconds(), retryAllowed, fire.feedGate.ballLeft());
    // The retry's pause: in the auto a short reverse, to back a stuck ball off before full power forward
    if (indexerRetry.paused()) { indexerCommand = robot.autonomous ? IndexerState.EJECT : IndexerState.STOP; }
    if (indexerRetry.fullPower()) { indexerPower = 1.0; }
    indexerPower = loadingIndexerPower(indexerCommand, loading, robot.autonomous, indexerPower);
    robot.indexer.setIndexerState(indexerCommand, indexerPower);
    Logger.recordOutput("Indexer/Retry/Paused", indexerRetry.paused());
    Logger.recordOutput("Indexer/Retry/FullPower", indexerRetry.fullPower());
    if (robot.spinOnlyOnFire && loading && !fire.loadingShooter.canLoad()) {
      robot.telemetry.addLine("LT: waiting for flywheels to stop");
    }
    boolean volleyBetweenBalls = fire.volleyShot && fire.volleyFeedGate.ballsLeft() > 0
        && !VolleyConstants.VOLLEY_HOLD_BETWEEN_BALLS;
    FeederState feederCommand = feederCommand(ejecting, loading || fire.loadingShooter.clearing(), shooting, feeding,
        fireRequested, fire.feedGate.feederEmpty(), volleyBetweenBalls);
    if (indexerRetry.paused()) { feederCommand = FeederState.STOP; }
    robot.feeder.setFeederState(feederCommand, feederPower);
  }

  /**
   * The feeder's command. Held back whenever nothing shoots, as Volley Tuning's always-aimed robot does: stopped,
   * the spinning wheels and the intake's push took balls out of the stack between shots in Lebot Teleop, and the
   * auto, aiming all the time, left it stopped after an empty volley while the indexer staged the next pickup at
   * full power: balls went out on the way to the Flower.
   */
  static FeederState feederCommand(boolean ejecting, boolean loadingOrClearing, boolean shooting, boolean feeding,
      boolean fireRequested, boolean feederEmpty, boolean volleyBetweenBalls) {
    if (ejecting) {
      return FeederState.EJECT;
    }
    if (loadingOrClearing) {
      return FeederState.HOLD; // Even if the last trial's feed gate says empty
    }
    if (shooting && feeding) {
      return FeederState.FEED;
    }
    if (shooting && fireRequested && (feederEmpty || volleyBetweenBalls)) {
      // Between a volley's balls the feeder stops, so the next ball isn't pulled back from the wheels
      return FeederState.STOP;
    }
    return FeederState.HOLD; // The last roller isolates the moving upstream queue while an aimed shot waits
  }

  /** Below this the flywheels count as stopped: a ball pushed into them doesn't fly. */
  static final double WHEELS_STOPPED_RPM = 100;

  /**
   * "Packed" and "full" clear on a ball leaving, B, or an LT press after LT was let go REARM_RELEASE_S or more, the
   * last only with the flywheels stopped: cleared with the wheels spinning, loading pushed the stack at its last
   * roller's power and shot the front ball out.
   */
  static boolean loadingLatchesClear(boolean newLtPress, double ltReleasedForS, boolean ejecting, boolean ballLeft,
      boolean wheelsSpinning) {
    return (newLtPress && ltReleasedForS >= StackPressure.REARM_RELEASE_S && !wheelsSpinning) || ejecting || ballLeft;
  }

  /** The intake's ease ends on an LT press after a real let-go, or once it runs free (nothing pushing back). */
  static boolean intakeEaseClears(boolean newLtPress, double ltReleasedForS, boolean runningFree) {
    return (newLtPress && ltReleasedForS >= StackPressure.REARM_RELEASE_S) || runningFree;
  }

  /**
   * The intake as LT, B and a full stack ask. A full stack stops it rather than ejecting: a 5th ball just waits at
   * the mouth, and in the auto the Pollen already taken stays in for the next volley.
   */
  static IntakeState intakeCommand(boolean full, boolean ejecting, boolean wanted) {
    if (full && !ejecting) {
      return IntakeState.STOP;
    }
    return IntakeSubsystem.automaticState(full, ejecting, wanted);
  }

  /**
   * In the auto the last roller keeps bringing balls up between shots, against the held-back feeder, as loading does:
   * stopped (STAGE), the next ball never reached the feeder in the 0.2-0.4 s feed pulses.
   */
  static IndexerState autoStage(IndexerState command, boolean autonomous) {
    return autonomous && command == IndexerState.STAGE ? IndexerState.LOAD : command;
  }

  /** The auto's agitation pauses only loading: a feed, an eject or a stop goes on as commanded. */
  static IndexerState agitated(IndexerState command, boolean paused) {
    return agitated(command, paused, false);
  }

  /** As above; with reverse the paused part runs the indexer backward, jiggling the stack, instead of stopping it. */
  static IndexerState agitated(IndexerState command, boolean paused, boolean reverse) {
    if (!paused || command != IndexerState.LOAD) {
      return command;
    }
    return reverse ? IndexerState.EJECT : IndexerState.STOP;
  }

  /**
   * LT loading runs the upstream rollers at LOAD_UPSTREAM_POWER: with IndexerServo3 added, two upstream rollers at
   * full power pushed the stack past the backward-held feeder and shot the front ball while loading. The auto keeps
   * full power to stage its next ball in time.
   */
  static double loadingIndexerPower(IndexerState command, boolean loading, boolean autonomous, double power) {
    boolean loadingCommand = command == IndexerState.LOAD || command == IndexerState.STAGE;
    return loading && !autonomous && loadingCommand ? Math.min(power, DriverConstants.LOAD_UPSTREAM_POWER) : power;
  }

  /** A packed stack loads with its last roller stopped, so nothing creeps the first ball into the wheels. */
  static IndexerState packedLoad(IndexerState command, boolean packed) {
    return packed && command == IndexerState.LOAD ? IndexerState.STAGE : command;
  }

  /** Intake protection only controls its motor; LT, eject and shot gates still control the indexer. */
  static IndexerState indexerCommand(boolean ejecting, boolean fireRequested, boolean feeding,
      double leftTrigger, boolean holdFire, boolean full) {
    if (ejecting) {
      return IndexerState.EJECT;
    }
    // Aim alone is not a request to run the indexer. An armed shot stages the upstream
    // queue through recovery gaps, with its independent last roller stopped until feeding.
    if (fireRequested && !holdFire) {
      return feeding ? IndexerState.FEED : IndexerState.STAGE;
    }
    return leftTrigger > Robot.TRIGGER_PRESSED && !full ? IndexerState.LOAD : IndexerState.STOP;
  }
}

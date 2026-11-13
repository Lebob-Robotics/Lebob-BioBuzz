package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

/** Pedro's tuning menu. Each procedure drives the robot: give it room. */
public final class Tuning {
  private Tuning() {}

  @Tuner(name = "Foresight Tuner")
  public static Procedure foresight() {
    return new ForesightTuner(Constants::tuningLocalizer, Constants::tuningDrivetrain);
  }

  @Tuner(name = "Tests")
  public static Procedure tests() {
    return new Tests(Constants::tuningDrivetrain, Constants::tuningLocalizer, Constants::algorithm);
  }
}

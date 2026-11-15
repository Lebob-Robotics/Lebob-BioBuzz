package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.EnumSet;
import java.util.function.Predicate;

/** Which gamepad buttons went down since the last {@link #update}: a press counts once, however long it is held. */
public final class Buttons {
  public enum Button {
    A(p -> p.a),
    B(p -> p.b),
    X(p -> p.x),
    Y(p -> p.y),
    LEFT_BUMPER(p -> p.left_bumper),
    BACK(p -> p.back),
    START(p -> p.start),
    DPAD_UP(p -> p.dpad_up),
    DPAD_DOWN(p -> p.dpad_down),
    DPAD_LEFT(p -> p.dpad_left),
    DPAD_RIGHT(p -> p.dpad_right),
    LEFT_STICK(p -> p.left_stick_button),
    RIGHT_STICK(p -> p.right_stick_button);

    private final Predicate<Gamepad> down;

    Button(Predicate<Gamepad> down) {
      this.down = down;
    }
  }

  private final EnumSet<Button> down = EnumSet.noneOf(Button.class);
  private final EnumSet<Button> pressed = EnumSet.noneOf(Button.class);

  /**
   * Once a loop. With buttons given, only those are read; the rest count as up until read, so INIT can watch its one
   * button and a press of another held through START still counts then.
   */
  public void update(Gamepad pad, Button... only) {
    pressed.clear();
    for (Button b : only.length == 0 ? Button.values() : only) {
      boolean now = b.down.test(pad);
      if (now && down.add(b)) {
        pressed.add(b);
      } else if (!now) {
        down.remove(b);
      }
    }
  }

  public boolean pressed(Button b) {
    return pressed.contains(b);
  }
}

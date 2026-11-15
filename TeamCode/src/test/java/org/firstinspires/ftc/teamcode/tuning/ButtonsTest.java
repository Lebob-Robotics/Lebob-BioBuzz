package org.firstinspires.ftc.teamcode.tuning;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.tuning.Buttons.Button;
import org.junit.Test;

public class ButtonsTest {
  @Test
  public void aPressCountsOnceHoweverLongItIsHeld() {
    Buttons buttons = new Buttons();
    Gamepad pad = new Gamepad();
    buttons.update(pad);
    assertFalse(buttons.pressed(Button.B));
    pad.b = true;
    buttons.update(pad);
    assertTrue(buttons.pressed(Button.B));
    buttons.update(pad);
    assertFalse(buttons.pressed(Button.B));
    pad.b = false;
    buttons.update(pad);
    pad.b = true;
    buttons.update(pad);
    assertTrue(buttons.pressed(Button.B));
  }

  @Test
  public void onlyTheButtonsGivenAreRead() {
    Buttons buttons = new Buttons();
    Gamepad pad = new Gamepad();
    pad.y = true;
    pad.start = true;
    buttons.update(pad, Button.Y); // As INIT watches Y
    assertTrue(buttons.pressed(Button.Y));
    assertFalse(buttons.pressed(Button.START));
    buttons.update(pad);
    assertFalse(buttons.pressed(Button.Y));
    assertTrue("Start held through START still counts", buttons.pressed(Button.START));
  }

  @Test
  public void eachButtonReadsItsOwnField() {
    Gamepad pad = new Gamepad();
    pad.a = pad.x = pad.left_bumper = pad.back = pad.dpad_up = pad.dpad_down = pad.dpad_left = pad.dpad_right = true;
    pad.left_stick_button = pad.right_stick_button = true;
    Buttons buttons = new Buttons();
    buttons.update(pad);
    for (Button b : Button.values()) {
      boolean expected = b != Button.B && b != Button.Y && b != Button.START;
      assertTrue(b + " read wrong", buttons.pressed(b) == expected);
    }
  }
}

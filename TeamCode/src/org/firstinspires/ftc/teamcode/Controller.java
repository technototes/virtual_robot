package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Controller {
  private Gamepad pad = null;
  private Telemetry telemetry = null;
  private String name = null;

  public Controller(Gamepad p, Telemetry tel, String nm) {
    pad = p;
    name = nm;
  }

  Direction dpad() {
    float X = 0, Y = 0;
    if (pad.dpad_down) {
      Y = -1;
    }
    if (pad.dpad_up) {
      Y = 1;
    }
    if (pad.dpad_left) {
      X = -1;
    }
    if (pad.dpad_right) {
      X = 1;
    }
    return getStick("d", X, Y);
  }

  public Direction lstick() {
    return getStick("l", pad.left_stick_x, pad.left_stick_y);
  }

  public Direction rstick() {
    return getStick("r", pad.right_stick_x, pad.right_stick_y);
  }

  private Direction getStick(String which, float X, float Y) {
    if (name != null) {
      // telemetry.addData(name + "-" + which + "> ", "X:%3.2f, Y:%3.2f", (double)X, (double)Y);
    }
    return new Direction(X, Y);
  }

  double ltrigger() {
    return (double) pad.left_trigger;
  }

  double rtrigger() {
    return (double) pad.right_trigger;
  }

  Button lbump() {
    return toButton(pad.left_bumper);
  }

  Button rbump() {
    return toButton(pad.right_bumper);
  }

  Button back() {
    return toButton(pad.back);
  }

  Button mode() {
    return toButton(pad.guide);
  }

  Button start() {
    return toButton(pad.start);
  }

  Button buttonA() {
    return toButton(pad.a);
  }

  Button buttonB() {
    return toButton(pad.b);
  }

  Button buttonX() {
    return toButton(pad.x);
  }

  Button buttonY() {
    return toButton(pad.y);
  }

  private Button toButton(boolean pressed) {
    return pressed ? Button.Pressed : Button.Released;
  }
}

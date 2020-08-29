package org.firstinspires.ftc.teamcode;

public enum Button {
  Pressed,
  Released;

  public boolean isPressed() {
    return this == Pressed;
  }

  public boolean isReleased() {
    return this == Released;
  }
}

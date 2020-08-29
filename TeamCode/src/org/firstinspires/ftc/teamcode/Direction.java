package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;

public class Direction {
  public double X, Y;

  public Direction(double x, double y) {
    X = x;
    Y = y;
  }
  //straight up is zero
  public Direction(double angle){

  }

  public static Direction None = new Direction(0.0, 0.0);

  public boolean isOnlyRight() {
    return (Math.abs(Y) < .3) && (X < -.7);
  }

  public boolean isOnlyLeft() {
    return (Math.abs(Y) < .3) && (X > .7);
  }

  public boolean isOnlyUp() {
    return (Math.abs(X) < .3) && (Y > .7);
  }

  public boolean isOnlyDown() {
    return (Math.abs(X) < .3) && (Y < -.7);
  }

  public boolean isRight() {
    return (X < -.5);
  }

  public boolean isLeft() {
    return (X > .5);
  }

  public boolean isUp() {
    return (Y > .5);
  }

  public boolean isDown() {
    return (Y < -.5);
  }

}

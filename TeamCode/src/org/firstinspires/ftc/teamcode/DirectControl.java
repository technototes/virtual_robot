package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.technototes.library.hardware.servo.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Direct Control")
public class DirectControl extends LinearOpMode {
  private static double FINEDRIVESPEED = 0.2;
  private TTRobot robot;
  private TTRobot robotForTest = null;
  private Controller control;
  private Controller driver;
  private XDriveManualControl manualCtrl;

  // This is the middle 'dead zone' of the analog sticks
  private static final double STICKDEADZONE = 0.05;
  // Triggers must be pushed at least this far
  private static final double TRIGGERTHRESHOLD = 0.25;

  void SetTestRobot(TTRobot testRobot) {
    robotForTest = testRobot;
  }

  @Override
  public void runOpMode() {
    // If you want telemetry, include a name as a string
    // If you don't want telemetry, pass a null:
    driver = new Controller(gamepad1, telemetry, "driver");
    control = new Controller(gamepad2, telemetry, "controller");
    robot = (robotForTest != null) ? robotForTest : new TTRobot(this, hardwareMap, telemetry);
    manualCtrl = new XDriveManualControl(robot, driver, control, telemetry);


    waitForStart();
    ElapsedTime sinceLastUsedGrabRotate = new ElapsedTime();
    ElapsedTime timeSinceStart = new ElapsedTime();
    ElapsedTime loopTime = new ElapsedTime();

    int curBrickHeight = -1;
    boolean liftOverrideDownEnabled = false;

    while (opModeIsActive()) {
      loopTime.reset();
      // Handle Grabber clutch
      if (driver.rtrigger() > TRIGGERTHRESHOLD) {
        robot.setClawPosition(ClawPosition.Open); // Open
      } else if (driver.ltrigger() > TRIGGERTHRESHOLD) {
        robot.setClawPosition(ClawPosition.Close); // Closed
      }
      // Grabber rotation
      final double grabRotationDebounceSecs = 0.25;
      if (sinceLastUsedGrabRotate.seconds() > grabRotationDebounceSecs) {
        if (driver.lbump() == Button.Pressed) {
          robot.rotateClaw(true);
          telemetry.addLine("rotateClaw true");
          sinceLastUsedGrabRotate.reset();
        } else if (driver.rbump() == Button.Pressed) {
          robot.rotateClaw(false);
          telemetry.addLine("rotateClaw false");
          sinceLastUsedGrabRotate.reset();
        }
      }

      // Override the linear slide limit switches
      boolean slideOverride = (control.rbump() == Button.Pressed) && (control.lbump() == Button.Pressed);
//      Direction ctrlDpad = control.dpad();
//      if (ctrlDpad.isLeft()) {
//        robot.setLinearSlideDirectionRyan(LinearSlideOperation.Extend, !slideOverride);
//      } else if (ctrlDpad.isRight()) {
//        robot.setLinearSlideDirectionRyan(LinearSlideOperation.Retract, !slideOverride);
//      } else {
//        robot.setLinearSlideDirectionRyan(LinearSlideOperation.None, !slideOverride);
//      }
      if (driver.buttonX().isPressed()) {
        robot.setLinearSlideDirectionRyan(LinearSlideOperation.Extend, !slideOverride);
      } else if (driver.buttonB().isPressed()) {
        robot.setLinearSlideDirectionRyan(LinearSlideOperation.Retract, !slideOverride);
      } else {
        robot.setLinearSlideDirectionRyan(LinearSlideOperation.None, !slideOverride);
      }

      Direction dcontrols = driver.dpad();
      if (dcontrols.isUp()) {
        robot.blockFlipper(FlipperPosition.Down);
      } else {
        robot.blockFlipper(FlipperPosition.Up);
      }

      if (dcontrols.isLeft()) {
        robot.capstone(-1);
      } else if (dcontrols.isRight()) {
        robot.capstone(1);
      } else {
        robot.capstone(0);
      }

      if ((control.ltrigger() > 0.8) && (control.rtrigger() > 0.8) &&
           control.rbump().isPressed() && control.lbump().isPressed()) {
        if (control.buttonX().isPressed()) {
          robot.lift.overrideDown();
          liftOverrideDownEnabled = true;
        } else {
          robot.lift.stop();
          robot.lift.ResetZero();
          curBrickHeight = -1;
          liftOverrideDownEnabled = false;
        }
      } else {
        // Released all the buttons simultaneously, so need to stop & reset the lift now
        if (liftOverrideDownEnabled) {
          robot.lift.stop();
          robot.lift.ResetZero();
          curBrickHeight = -1;
          liftOverrideDownEnabled = false;
        }

        // More automated control of the lift:
        // Y for 'up a brick'
        // X for 'down a brick'
        // A for 'position current brick to place'
        // B for 'grab a brick'
        if (control.buttonA().isPressed()) {
          robot.lift.SetBrickWait();
        } else if (control.buttonY().isPressed() && curBrickHeight < LiftControl.MAX_BRICK_HEIGHT) {
          robot.lift.LiftBrickWait(++curBrickHeight);
        } else if (driver.buttonX().isPressed() && curBrickHeight > 0) {
          robot.lift.LiftBrickWait(--curBrickHeight);
        } else if (driver.buttonB().isPressed()) {
          robot.lift.AcquireBrickWait();
          curBrickHeight = -1;
        }
      }
      if(driver.buttonY().isPressed()){
        robot.lift.up();
      } else if(driver.buttonA().isPressed()){
        robot.lift.overrideDown();
      } else{
        robot.lift.stop();
      }

      if (driver.ltrigger() >  0.8 && driver.rtrigger() > 0.8 && driver.rbump().isPressed() && driver.lbump().isPressed()) {
        robot.initGyro();
      }

      telemetry.addData("Left trigger pos: ", driver.ltrigger());
      telemetry.addData("Right trigger pos: ", driver.rtrigger());
      // This is just steering
      manualCtrl.Steer();

      telemetry.addLine(String.format("Timing: %.1f, %.1f", timeSinceStart.seconds(), loopTime.seconds()));
      telemetry.update();
    }
  }
}

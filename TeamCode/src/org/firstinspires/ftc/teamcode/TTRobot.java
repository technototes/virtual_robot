package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class TTRobot implements IRobot {
  // Scaling values

  // the power of the linear slide
  private static final double LINEARSLIDEPOWER = 0.5;

  // Claw grab positions
  private static final double CLAWOPENPOSITION = 0.7;
  private static final double CLAWCLOSEPOSITION = 0;

  // Block flipper / baseplate grabber positions
  private static final double BLOCK_FLIPPER_UP_POSITION = 0.75;
  private static final double BLOCK_FLIPPER_DOWN_POSITION = 0.15;

  // Distance speeds in cm for fast auto drive functions
  private static final double TURBODISTANCE = 65;
  private static final double SNAILDISTANCE = 15;

  // Unused stuff
  // the grab rotation position for snapping to horizontal or vertical
  private static final double GRABBERPOSITIONCUTOFF = 0.25;
  // the grab rotation 'horizontal' position
  private static final double HORIZONTALGRABBERPOSITION = 0.0;
  // the grab rotation 'vertical' position
  private static final double VERTICALGRABBERPOSITION = 0.5;

  enum LinearSlidePosition {
    In,
    Middle,
    Out
  }

  private boolean t;

  private LinearSlidePosition slidePosition = LinearSlidePosition.In;

  private CRServo slide = null;
  public LiftControl lift = null;
  private Servo turn = null;
  private Servo claw = null;
  private Servo blockFlipper = null;
  private CRServo cap = null;
  private ColorSensor sensorColorBottom = null;
  private DistanceSensor sensorRangeFront = null;
  private DistanceSensor sensorRangeRear = null;
  private DistanceSensor sensorRangeLeft = null;
  private DistanceSensor sensorRangeRight = null;

  private XDrive driveTrain = null;
  private Telemetry telemetry = null;
  private LinearOpMode opMode = null;
  // Stuff for the on-board "inertial measurement unit" (aka gyro)
  // The IMU sensor object
  private BNO055IMU imu;
  // State used for updating telemetry
  private Orientation angles;

  private static boolean UNTESTED = false;

  // This is an 'opMode aware' sleep: It will stop if you hit 'stop'!
  public final void sleep(long milliseconds) {
    telemetry.addData("Sleeping!", milliseconds);

    try {
      if (UNTESTED) {
        ElapsedTime tm = new ElapsedTime();
        while (tm.milliseconds() < milliseconds && opMode.opModeIsActive()) {
          Thread.sleep(Math.min(milliseconds - (long) tm.milliseconds(), 50));
        }
      } else {
        Thread.sleep(milliseconds);
      }
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }
  }

  public TTRobot(LinearOpMode op, HardwareMap hardwareMap, Telemetry tel) {
    telemetry = tel;
    opMode = op;
    // Get handles to all the hardware
    slide = hardwareMap.get(CRServo.class, "slide");
    turn = hardwareMap.get(Servo.class, "grabTurn");
    claw = hardwareMap.get(Servo.class, "claw");
    blockFlipper = hardwareMap.get(Servo.class, "blockFlipper");
    cap = hardwareMap.get(CRServo.class, "cap");
    sensorRangeFront = hardwareMap.get(DistanceSensor.class, "sensorRangeFront");
    sensorRangeRear = hardwareMap.get(DistanceSensor.class, "sensorRangeRear");
    sensorRangeLeft = hardwareMap.get(DistanceSensor.class, "sensorRangeLeft");
    sensorRangeRight = hardwareMap.get(DistanceSensor.class, "sensorRangeRight");

    DcMotor lLiftMotor = hardwareMap.get(DcMotor.class, "motorLiftLeft");
    DcMotor rLiftMotor = hardwareMap.get(DcMotor.class, "motorLiftRight");
    lift = new LiftControl(op, lLiftMotor, rLiftMotor);
    sensorColorBottom = hardwareMap.get(ColorSensor.class, "sensorColorBottom");

    DcMotor flMotor = hardwareMap.get(DcMotor.class, "motorFrontLeft");
    DcMotor frMotor = hardwareMap.get(DcMotor.class, "motorFrontRight");
    DcMotor rlMotor = hardwareMap.get(DcMotor.class, "motorRearLeft");
    DcMotor rrMotor = hardwareMap.get(DcMotor.class, "motorRearRight");
    driveTrain = new XDrive(flMotor, frMotor, rlMotor, rrMotor);

    // Setup the IMU
    // Set up the parameters with which we will use our IMU. Note that integration
    // algorithm here just reports accelerations to the logcat log; it doesn't actually
    // provide positional information.
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    parameters.calibrationDataFile =
      "AdafruitIMUCalibration.json"; // see the calibration sample opmode
    imu = hardwareMap.get(BNO055IMU.class, "imu1");
    imu.initialize(parameters);

    // Calibrate
    // make lift motors work together: they're facing opposite directions
    lLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    rLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    // TODO: Add initialization / calibration for the slide and lift?

    // Shamelessly copied from example code...
    //sleep(2000);
    // Start the logging of measured acceleration


    //set grabber rotation to be centered
    centerClaw();
  }

  // Linear slide stuff:
  public boolean slideSwitchSignaled() {
    // Wound up not using this...
    return false;
  }

  public void setLinearSlideDirectionRyan(LinearSlideOperation operation, boolean override) {
    double power = 0;
    switch (operation) {
      case Extend:
        power = LINEARSLIDEPOWER;
        break;
      case Retract:
        power = -LINEARSLIDEPOWER;
        break;
      case None:
        power = 0;
        break;
    }
    slide.setPower(power);
    while (!override && power != 0 && slideSwitchSignaled() && opMode.opModeIsActive()) {
      slide.setPower(-power);
      sleep(1);
    }
  }

  private LinearSlidePosition currentPos = LinearSlidePosition.In;

  public void setLinearSlideDirection(LinearSlideOperation operation, boolean override) {
    double power = 0;
    switch (operation) {
      case Extend:
        power = LINEARSLIDEPOWER;
        break;
      case Retract:
        power = -LINEARSLIDEPOWER;
        break;

    }

    switch (slidePosition) {
      case In:
        switch (operation) {
          case Extend:
            if (!slideSwitchSignaled()) {
              slidePosition = LinearSlidePosition.Middle;
            }
            break;
          case Retract:
            if (override)
              break;
            //otherwise do not move

          default:
            // Do nothing
            power = 0;
            break;
        }
        break;

      case Middle:
        // Hit a limit
        if (!override && slideSwitchSignaled()) {
          // Stop the slide

          power = 0;

          // Update the state
          switch (operation) {
            case Extend:
              slidePosition = LinearSlidePosition.Out;
              break;
            case Retract:
              slidePosition = LinearSlidePosition.In;
              break;
          }
        }
        break;

      case Out:
        switch (operation) {
          case Retract:
            if (!slideSwitchSignaled()) {
              slidePosition = LinearSlidePosition.Middle;
            }
            break;

          case Extend:
            if (override)
              break;
            //otherwise do not move

          default:
            // Do nothing
            power = 0;
            break;
        }
        break;
    }

    slide.setPower(power);
  }

  // Grabber stuff:
  private static final double CLAW_LEFT_VAL = 0;
  private static final double CLAW_CENTER_VAL = 0.3;
  private static final double CLAW_RIGHT_VAL = 1.0;

  public enum CurrentClawPosition {
    LEFT, CENTER, RIGHT;
  }

  public void centerClaw(){
    turn.setPosition(CLAW_CENTER_VAL);
  }
  private CurrentClawPosition curPos = CurrentClawPosition.CENTER;

  public void rotateClaw(boolean increment) {
    if(curPos == CurrentClawPosition.CENTER){
      if(increment){
        turn.setPosition(CLAW_RIGHT_VAL);
        curPos = CurrentClawPosition.RIGHT;
      } else{
        turn.setPosition(CLAW_LEFT_VAL);
        curPos = CurrentClawPosition.LEFT;
      }
    } else if(curPos == CurrentClawPosition.LEFT){
      if(increment){
        turn.setPosition(CLAW_CENTER_VAL);
        curPos = CurrentClawPosition.CENTER;
      }
    } else{
      if(!increment){
        turn.setPosition(CLAW_CENTER_VAL);
        curPos = CurrentClawPosition.CENTER;
      }
    }
    //telemetry.addData("val", turn.getPosition());
    //telemetry.update();
  }

  public void setClawPosition(ClawPosition position) {
    switch (position) {
      case Open:
        claw.setPosition(CLAWOPENPOSITION);
        break;
      case Close:
        claw.setPosition(CLAWCLOSEPOSITION);
        break;
    }
    telemetry.addData("Claw: ", position.toString());
  }

  // Lift stuff:
  enum LiftState {
    Above,
    At,
    Below
  }

  public void blockFlipper(FlipperPosition pos) {
    switch (pos) {
      case Down:
        blockFlipper.setPosition(BLOCK_FLIPPER_DOWN_POSITION);
        break;
      case Up:
      default:
        blockFlipper.setPosition(BLOCK_FLIPPER_UP_POSITION);
        break;
    }
  }
  public com.technototes.library.hardware.servo.Servo sss = new com.technototes.library.hardware.servo.Servo(blockFlipper);


  public void capstone(double speed) {
    cap.setPower(-speed);
  }

  // 0 = facing toward the driver (6 O'Clock)
  // 90 = 9 O'clock
  // -90 = 3:00
  public double gyroHeading() {
    Orientation angles1 =
      imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    return -AngleUnit.DEGREES.fromUnit(angles1.angleUnit, angles1.firstAngle + 180);
  }

  // 0 = facing away from driver (12 O'Clock)
  // 90 degrees: 3:00
  // -90 degrees: 9:00
  public double gyroHeading2() {
    Orientation angles1 =
      imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    return -AngleUnit.DEGREES.fromUnit(angles1.angleUnit, angles1.firstAngle);
  }

  void setServoDirection(Servo.Direction direction) {
    turn.setDirection(direction);
  }

  void setServoPosition(double position) {
    turn.setPosition(position);
  }

  // Distance sensing
  public double frontDistance() {
    return sensorRangeFront.getDistance(DistanceUnit.CM);
  }

  public double rearDistance() {
    return sensorRangeRear.getDistance(DistanceUnit.CM);
  }

  public double leftDistance() {
    return sensorRangeLeft.getDistance(DistanceUnit.CM);
  }

  public double rightDistance() {
    return sensorRangeRight.getDistance(DistanceUnit.CM);
  }

  // Drive train:

  public void speedSnail() {
    driveTrain.setSpeed(XDrive.DriveSpeed.Snail);
  }

  public void speedTurbo() {
    driveTrain.setSpeed(XDrive.DriveSpeed.Turbo);
  }

  public void speedNormal() {
    driveTrain.setSpeed(XDrive.DriveSpeed.Normal);
  }

  // set nearestSnap to true to snap to nearest 90 dgree angle, or set nearestSnap to false and
  // input angle to snap to.
  public double snapToAngle(double gyroAngle) {
    double test = 0.0;
    if (gyroAngle > 50 && gyroAngle < 130) {
      test = 90 - gyroAngle;
    } else if (gyroAngle > 140 && gyroAngle < 180) {
      test = 180 - gyroAngle;
    } else if (gyroAngle > -180 && gyroAngle < -140) {
      test = -180 - gyroAngle;
    } else if (gyroAngle > -130 && gyroAngle < -50) {
      test = -90 - gyroAngle;
    } else if (gyroAngle > -40 && gyroAngle < 40) {
      test = 0 - gyroAngle;
    }
    return test;
  }

  //for autonomous only
  public void toAngle(double to) {
    if (to > 0) {
      while (to > gyroHeading() && opMode.opModeIsActive()) {
        Direction dir = new Direction(1, 0);
        joystickDrive(Direction.None, dir, 0);
      }
    } else {
      while (to < gyroHeading() && opMode.opModeIsActive()) {
        Direction dir = new Direction(-1, 0);
        joystickDrive(Direction.None, dir, 0);
      }
    }
  }

  // Snap the robot to the closest 90 degree angle
  public double snap() {
    double curr = gyroHeading();
    double newangle = snapToAngle(curr);
    telemetry.addData("Snap:", String.format("curr: %3.3f new: %3.3f", curr, newangle));
    return scaledSnap(newangle);
    //return snap(newangle); replaced with above scaledSnap
  }

  // Turn the robot to a specific angle
  private double snap(double targetAngle) {
    if (targetAngle < -25) {
      return -1.0;
    } else if (targetAngle > 25) {
      return 1.0;
    } else if (targetAngle < -5) {
      return -.1;
    } else if (targetAngle > 5) {
      return .1;
    }
    return 0;
  }

  private double scaledSnap(double targetAngle) {
    double angleMag = Math.abs(targetAngle);
    double motorMag = 0.0;
    if (angleMag > 35.0) {
      motorMag = 1.0;
    } else if (angleMag > 25.0) {
      motorMag = .7;
    } else if (angleMag > 15.0) {
      motorMag = .4;
    } else if (angleMag > 5.0) {
      motorMag = .2;
    } else if (angleMag > 0.0) {
      motorMag = 0.0;
    }
    if (targetAngle < 0.0) {
      motorMag = -motorMag;
    }
    return motorMag;
  }

  public void stop() {
    driveTrain.stop();
  }

  // leave gyroAngle at zero to set relative angle
  public void joystickDrive(Direction j1, Direction j2, double gyroAngle) {
    driveTrain.setStickVector(j1.X, j1.Y, j2.X, gyroAngle);
    telemetry.addData("control rstick X: ", j2.X);
  }

  public void timeDrive(double speed, double time, double angle) {

    ElapsedTime runTime = new ElapsedTime();
    while (runTime.seconds() < time && opMode.opModeIsActive()) {
      driveTrain.timeDrive(speed, time, angle, gyroHeading());
    }
  }

  public void vectorDrive(double speed, double angle) {
    driveTrain.setDriveVector(speed, angle, gyroHeading());
  }

  public void lineDrive(double speed, double time, double angle) {
    driveTrain.setDriveVector(speed, angle, gyroHeading());
    ElapsedTime tm = new ElapsedTime();
    int red, blue;
    do {
      sleep(10);
      red = sensorColorBottom.red();
      blue = sensorColorBottom.blue();
    } while (tm.seconds() < time && !(Math.abs(red - blue) > 50) && opMode.opModeIsActive());
    driveTrain.stop();
  }

  public void driveToLine(double speed, double direction) {
    lineDrive(speed, 5, direction);
  }

  // This will travel toward the rear until it gets to 'dist'
  public void distRearDrive(double speed, double dist) {
    ElapsedTime tm = new ElapsedTime();
    double curDistance = 0;
    do {
      curDistance = rearDistance();
      telemetry.addData("Current Distance", curDistance);
      telemetry.update();
      double dir = (dist < curDistance) ? 1 : -1;
      double speedMult = (Math.abs(dist - curDistance) > 10) ? 1.0 : 0.5;
      driveTrain.setStickVector(XDrive.DriveSpeed.Normal, 0, dir * speed * speedMult, 0, gyroHeading());
      sleep(10);
    } while (Math.abs(curDistance - dist) > 2 && tm.seconds() < 3.0 && opMode.opModeIsActive());
    driveTrain.stop();
  }

  public void syncTurn(double angle, double time) {

    ElapsedTime runTime = new ElapsedTime();
    runTime.reset();
    while (runTime.seconds() < time && opMode.opModeIsActive()) {
      if (gyroHeading2() > angle + 5) {
        joystickDrive(Direction.None, new Direction(-0.5, 0), gyroHeading2());
      } else if (gyroHeading2() < angle - 5) {
        joystickDrive(Direction.None, new Direction(0.5, 0), gyroHeading2());
      } else {
        joystickDrive(Direction.None, new Direction(0, 0), gyroHeading2());
        time = 0;
      }
      telemetry.addData("gyro:", gyroHeading());
      telemetry.addData("gyro2:", gyroHeading2());
      telemetry.update();
    }
    //new code
    //._. ._.
    // \_(0.0)_/

  }

  public void distLeftDrive(double speed, double angle, double leftDist) {
    // TODO: Check this angle
    driveTrain.setDriveVector(speed, angle, gyroHeading());
    ElapsedTime tm = new ElapsedTime();
    do {
      sleep(10);
    } while (getCappedRange(sensorRangeLeft, 1500) > leftDist && tm.seconds() < 10.0 && opMode.opModeIsActive());
    driveTrain.stop();
  }

  public void distRightDrive(double speed, double angle, double rightDist) {
    // TODO: Check this angle
    driveTrain.setDriveVector(speed, angle, gyroHeading());

    ElapsedTime tm = new ElapsedTime();
    tm.reset();
    do {
      sleep(10);
    } while (Math.abs(getCappedRange(sensorRangeRight, 1500) - rightDist) > 2 && tm.seconds() < 5.0 && opMode.opModeIsActive());
    driveTrain.stop();
  }

  // Adaptive autonomous driving stuff:


  public void setTurningSpeed(double angleDelta) {
    if (Math.abs(angleDelta) < 35) {
      speedSnail();
    } else if (Math.abs(angleDelta) > 130) {
      speedTurbo();
    }else{
      speedNormal();
    }
  }

  // Turn to the angle specified
  // FYI: 0 is facing 'away' from the driver
  // 90 == 3:00, -90 == 9:00, 0 == 6:00
  public void fastSyncTurn(double angle, double time) {
    ElapsedTime runTime = new ElapsedTime();
    runTime.reset();
    while (opMode.opModeIsActive() &&
      runTime.seconds() < time &&
      Math.abs(gyroHeading2() - angle) > 4) {
      if (gyroHeading2() > angle + 2) {
        setTurningSpeed(gyroHeading2() - angle);
        joystickDrive(Direction.None, new Direction(-0.7, 0), gyroHeading2());
      } else if (gyroHeading2() < angle - 2) {
        setTurningSpeed(angle - gyroHeading2());
        joystickDrive(Direction.None, new Direction(0.7, 0), gyroHeading2());
      }
      telemetry.addData("gyro:", gyroHeading());
      telemetry.addData("gyro2:", gyroHeading2());
      telemetry.update();
    }
    stop();
  }

  //turn and drive
  public void turnAndDrive(int angle, double speed, double driveAngle){
    driveAngle = Math.toRadians(driveAngle+90);
    while(Math.abs(gyroHeading2()-angle) > 5) {
      if (angle < gyroHeading2() - 5) {
        driveTrain.setSpeed(XDrive.DriveSpeed.Normal);
        driveTrain.setStickVector(Math.cos(driveAngle), Math.sin(driveAngle), -0.5, gyroHeading2());
      } else if (angle > gyroHeading2() + 5) {
        driveTrain.setSpeed(XDrive.DriveSpeed.Normal);
        driveTrain.setStickVector(Math.cos(driveAngle), Math.sin(driveAngle), 0.5, gyroHeading2());
      }
    }
    driveTrain.stop();
    //fastSyncTurn(angle, 1);
  }


  // This will travel toward the rear until it gets to 'dist'
  public void fastRearDrive(double dist) {
    // Update: No attention should be paid to 'speed'
    // Just drive and slow down when we get slow to the target
    ElapsedTime tm = new ElapsedTime();
    double curDistance = rearDistance();

    fastSyncTurn(0, 2);

    while (opMode.opModeIsActive() && Math.abs(curDistance - dist) > 4 && tm.seconds() < 3.0) {
      telemetry.addData("Current Distance", curDistance);
      telemetry.update();
      double dir = (dist < curDistance) ? 1 : -1;
      double magnitude = Math.abs(dist - curDistance);
      if (magnitude < 30) {
        speedSnail();
      } else if (magnitude < TURBODISTANCE) {
        speedNormal();
      } else {
        speedTurbo();
      }

      double heading = gyroHeading();

      //double turn = (heading < 0) ? .5 : -.5;
      //Direction rotation = new Direction((Math.abs(heading) > 175) ? turn : 0, 0);
      joystickDrive(new Direction(0, dir), Direction.None, heading);
      curDistance = rearDistance();
      sleep(10);
    }

    driveTrain.stop();
    speedNormal();
  }

// This will travel toward the front until it gets to 'dist'
  public void fastFrontDrive(double dist) {
    // Update: No attention should be paid to 'speed'
    // Just drive and slow down when we get slow to the target
    ElapsedTime tm = new ElapsedTime();
    double curDistance = frontDistance();

    fastSyncTurn(0, 2);

    while (opMode.opModeIsActive() && Math.abs(curDistance - dist) > 4 && tm.seconds() < 3.0) {
      telemetry.addData("Current Distance", curDistance);
      telemetry.update();
      double dir = (dist < curDistance) ? 1 : -1;
      double magnitude = Math.abs(dist - curDistance);
      if (magnitude < 30) {
        speedSnail();
      } else if (magnitude < TURBODISTANCE) {
        speedNormal();
      } else {
        speedTurbo();
      }

      double heading = gyroHeading();

      //double turn = (heading < 0) ? .5 : -.5;
      //Direction rotation = new Direction((Math.abs(heading) > 175) ? turn : 0, 0);
      joystickDrive(new Direction(0, dir), Direction.None, heading);
      curDistance = frontDistance();
      sleep(10);
    }

    driveTrain.stop();
    speedNormal();
  }



  public void fastLeftDrive(double dist) {
    // Update: No attention should be paid to 'speed'
    // Just drive and slow down when we get slow to the target
    ElapsedTime tm = new ElapsedTime();
    double curDistance = leftDistance();

    fastSyncTurn(0, 2);

    while (opMode.opModeIsActive() && Math.abs(curDistance - dist) > 4 && tm.seconds() < 3.0) {
      telemetry.addData("Current Distance", curDistance);
      telemetry.update();
      double dir = (dist < curDistance) ? -1 : 1;
      double magnitude = Math.abs(dist - curDistance);
      if (magnitude < SNAILDISTANCE) {
        speedSnail();
      } else if (magnitude < TURBODISTANCE) {
        speedNormal();
      } else {
        speedTurbo();
      }

      double heading = gyroHeading();
      //double turn = (heading < 0) ? .5 : -.5;
      //Direction rotation = new Direction((Math.abs(heading) > 175) ? turn : 0, 0);
      joystickDrive(new Direction(dir, 0), Direction.None, heading);
      curDistance = leftDistance();
      sleep(10);
    }

    driveTrain.stop();
    speedNormal();
  }

  public void fastRightDrive(double dist) {
    // Update: No attention should be paid to 'speed'
    // Just drive and slow down when we get slow to the target
    ElapsedTime tm = new ElapsedTime();
    double curDistance = rightDistance();
    fastSyncTurn(0, 2);
    while (opMode.opModeIsActive() && Math.abs(curDistance - dist) > 4 && tm.seconds() < 3.0) {
      telemetry.addData("Current Distance", curDistance);
      telemetry.update();
      double dir = (dist < curDistance) ? -1 : 1;
      double magnitude = Math.abs(dist - curDistance);
      if (magnitude < SNAILDISTANCE) {
        speedSnail();
      } else if (magnitude < TURBODISTANCE) {
        speedNormal();
      } else {
        speedTurbo();
      }

      // This should be "close" to 0 degrees
      double heading = gyroHeading2();
      Direction rotation = new Direction(-heading / 100, 0);
      joystickDrive(new Direction(dir, 0), rotation, heading);
      curDistance = rightDistance();
      sleep(10);
    }

    driveTrain.stop();
    speedNormal();
  }

  // This attempts to drive in a straight(ish) line toward a corner
  // This will travel toward the rear until it gets to 'dist'


  // This attempts to drive in a straight(ish) line toward a corner
  public void distRearLeftDrive(double speed, double rearDist, double leftDist) {
    double rDist, lDist;
    ElapsedTime tm = new ElapsedTime();
    fastSyncTurn(0, 2);
    double magnitude;
    do {

      // Let's figure out what angle to drive toward to make a straightish line
      // TODO: I have no idea if this is the proper angle or not
      // TODO: Might need to do something like 90 - angle
      rDist = getCappedRange(sensorRangeRear, 1500);
      lDist = getCappedRange(sensorRangeLeft, 1500);
      //double dir = (rearDist < rDist) ? -1 : 1;
      magnitude = (Math.abs(rearDist - rDist)+Math.abs(leftDist - lDist))/2;
      if (magnitude < SNAILDISTANCE) {
        speedSnail();
      } else if (magnitude < TURBODISTANCE) {
        speedNormal();
      } else {
        speedTurbo();
      }
      //double angle = Math.atan2(rDist, rtDist);
      //angle = AngleUnit.DEGREES.fromRadians(angle);
      if(rDist < rearDist && lDist > leftDist){
        //w
        joystickDrive(new Direction((lDist - leftDist) / (rDist - rearDist), (rDist - rearDist) / (lDist - leftDist)), new Direction(0, 0), gyroHeading());
      }else if(rDist > rearDist && lDist < leftDist){
        joystickDrive(new Direction(-(lDist - leftDist) / (rDist - rearDist), -(rDist - rearDist) / (lDist - leftDist)), new Direction(0, 0), gyroHeading());
      }else if(rDist < rearDist && lDist < leftDist){
        joystickDrive(new Direction((lDist - leftDist) / (rDist - rearDist), (rDist - rearDist) / (lDist - leftDist)), new Direction(0, 0), gyroHeading());
      }else{
        //w
        joystickDrive(new Direction(-(lDist - leftDist) / (rDist - rearDist), (rDist - rearDist) / (lDist - leftDist)), new Direction(0, 0), gyroHeading());
      }
      //fastSyncTurn(0, 0.01);
      telemetry.addData("rdist ", rDist);
      telemetry.addData("ldist ", lDist);
      telemetry.update();

    } while (magnitude > 7 && tm.seconds() < 10.0 && opMode.opModeIsActive());
    driveTrain.stop();
  }

  // This attempts to drive in a straight(ish) line toward a corner
  public void distRearRightDrive(double speed, double rearDist, double rightDist) {
    double rDist, rtDist;
    ElapsedTime tm = new ElapsedTime();
    fastSyncTurn(0, 2);
    double magnitude;
    do {
      // Let's figure out what angle to drive toward to make a straightish line
      // TODO: I have no idea if this is the proper angle or not
      // TODO: Might need to do something like 90 - angle
      rDist = getCappedRange(sensorRangeRear, 1500);
      rtDist = getCappedRange(sensorRangeRight, 1500);
      //double dir = (rearDist < rDist) ? -1 : 1;
      magnitude = (Math.abs(rearDist - rDist)+Math.abs(rightDist - rDist))/2;
      if (magnitude < SNAILDISTANCE) {
        speedSnail();
      } else if (magnitude < TURBODISTANCE) {
        speedNormal();
      } else {
        speedTurbo();
      }
      //double angle = Math.atan2(rDist, rtDist);
      //angle = AngleUnit.DEGREES.fromRadians(angle);
      if(rDist > rearDist && rtDist > rightDist){
        joystickDrive(new Direction((rtDist - rightDist) / (rDist - rearDist), (rDist - rearDist) / (rtDist - rightDist)), new Direction(0, 0), gyroHeading());
      }else if(rDist > rearDist && rtDist < rightDist){
        joystickDrive(new Direction((rtDist - rightDist) / (rDist - rearDist), -(rDist - rearDist) / (rtDist - rightDist)), new Direction(0, 0), gyroHeading());
      }else if(rDist < rearDist && rtDist < rightDist){
        joystickDrive(new Direction(-(rtDist - rightDist) / -(rDist - rearDist), (rDist - rearDist) / (rtDist - rightDist)), new Direction(0, 0), gyroHeading());
      }else{
        joystickDrive(new Direction(-(rtDist - rightDist) / (rDist - rearDist), (rDist - rearDist) / (rtDist - rightDist)), new Direction(0, 0), gyroHeading());
      }

      sleep(10);
      telemetry.addData("rdist ", rDist);
      telemetry.addData("rtdist ", rtDist);
      telemetry.update();

    } while(magnitude > 7 && tm.seconds() < 10.0 && opMode.opModeIsActive());
    //fastSyncTurn(0, 2);
    driveTrain.stop();
  }

  private double getCappedRange(DistanceSensor sens, double cap) {
    double res = sens.getDistance(DistanceUnit.CM);
    return Range.clip(res, 0.01, cap);
  }

  public void driveWallRear(double speed, double time, double angle, double distance) {
    double gyroAngle = gyroHeading();

    if (rearDistance() < distance && (angle < 180 && angle > 0)) {

      gyroAngle = gyroHeading() + 3;
    } else if (rearDistance() > distance && (angle > 180 && angle < 260)) {
      gyroAngle = gyroHeading() - 3;
    } else {
      gyroAngle = gyroHeading();
    }


    driveTrain.timeDrive(speed, time, angle, gyroHeading());
  }

  // Given a target angle & a current angle,
  // returns the direction & magnitude to turn to get to the target angle
  // This normalizes "I'm currently at 175, I need to go to -175"
  public static double goToAngle(double target, double current) {
    double delta = (target % 360) - (current % 360.0);
    if (delta > 180) {
      return delta - 360;
    } else if (delta < -180) {
      return 360 + delta;
    } else {
      return delta;
    }
  }

  public void initGyro() {
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    imu.initialize(parameters);
  }
}

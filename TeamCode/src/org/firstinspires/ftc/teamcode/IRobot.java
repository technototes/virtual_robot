package org.firstinspires.ftc.teamcode;

public interface IRobot {
    // This is the 'center' space on the controllers, used by the XDrive code, to not move the bot
    // Feel free to change it: The bigger it is, the more you have to push the sticks before the
    // robot moves
    public static final double STICK_DEAD_ZONE = 0.05;

    // This is an opModeActive aware sleep
    public void sleep(long milliseconds);

    // 0 = facing toward the driver (6 O'Clock)
    // 90 = 9 O'clock
    // -90 = 3:00
    public double gyroHeading();

    // 0 = facing away from driver (12 O'Clock)
    // 90 degrees: 3:00
    // -90 degrees: 9:00
    public double gyroHeading2();

    // Drive train:

    public void speedSnail();

    public void speedTurbo();

    public void speedNormal();

    // Snap the robot to the closest 90 degree angle
    public double snap();

    // Stop the phone
    public void stop();

    // leave gyroAngle at zero to set relative angle
    public void joystickDrive(Direction j1, Direction j2, double gyroAngle);

    public void timeDrive(double speed, double time, double angle);

    public void lineDrive(double speed, double time, double angle);

    public void driveToLine(double speed, double direction);

    // Turn to the angle specified
    // FYI: 0 is facing 'away' from the driver
    // 90 == 3:00, -90 == 9:00, 0 == 6:00
    public void fastSyncTurn(double angle, double time);

    public void initGyro();

}

package org.firstinspires.ftc.teamcode.newcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.technototes.library.structure.TeleOpCommandOpMode;
import org.firstinspires.ftc.teamcode.newcode.OI;
import org.firstinspires.ftc.teamcode.newcode.Robot;

@TeleOp(name = "test yay poggers")
public class TestOpMode extends TeleOpCommandOpMode {
    public OI oi;
    public Robot robot;

    @Override
    public void beginInit() {
        robot = new Robot(hardwareMap, telemetry);
        oi = new OI(driverGamepad, codriverGamepad, robot);
        //CommandScheduler.getRunInstance().schedule(new DriveCommand(robot.drivebaseSubsystem, driverGamepad.leftStick.x, driverGamepad.leftStick.y, driverGamepad.rightStick.x));

    }

    @Override
    public void runLoop() {
        robot.drivebaseSubsystem.joystickDrive(driverGamepad.leftStick.getYAxis(), driverGamepad.leftStick.getXAxis(), driverGamepad.rightStick.getYAxis(), robot.hardware.imu.gyroHeading());
        //System.out.println(robot.hardware.gyroHeading2());
    }
}

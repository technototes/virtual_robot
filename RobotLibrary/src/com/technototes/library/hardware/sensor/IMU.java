package com.technototes.library.hardware.sensor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.technototes.library.logging.Log;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class IMU extends Sensor<BNO055IMU> {
    public BNO055IMU.Parameters parameters;
    public IMU(BNO055IMU d) {
        super(d);
        parameters = new BNO055IMU.Parameters();
        degrees();
        device.initialize(parameters);

    }
    public IMU(String d) {
        super(d);
        parameters = new BNO055IMU.Parameters();
        degrees();
        device.initialize(parameters);

    }

    public IMU degrees(){
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        return this;
    }

    public IMU radians(){
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        return this;
    }

    @Log
    @Override
    public double getSensorValue() {
        return gyroHeading();
    }

    public double gyroHeading() {
        Orientation angles1 =
                device.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                        device.getParameters().angleUnit == BNO055IMU.AngleUnit.DEGREES? AngleUnit.DEGREES : AngleUnit.RADIANS);
        return -AngleUnit.DEGREES.fromUnit(angles1.angleUnit, angles1.firstAngle);
    }
}

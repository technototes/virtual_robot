package com.technototes.library.hardware.sensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RangeSensor extends Sensor<com.qualcomm.robotcore.hardware.DistanceSensor> {

    private DistanceUnit distanceUnit;

    public RangeSensor(com.qualcomm.robotcore.hardware.DistanceSensor d) {
        super(d);
    }

    @Override
    public double getSensorValue() {
        return device.getDistance(distanceUnit);
    }

    public double getSensorValue(DistanceUnit d) {
        return device.getDistance(d);
    }

    public DistanceUnit getDistanceUnit() {
        return distanceUnit;
    }

    public void setDistanceUnit(DistanceUnit d) {
        distanceUnit = d;
    }
}

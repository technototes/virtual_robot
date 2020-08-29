package com.technototes.library.hardware.sensor;

import com.qualcomm.robotcore.hardware.DigitalChannel;

public class DigitalSensor extends Sensor<DigitalChannel> {
    public DigitalSensor(DigitalChannel d) {
        super(d);
    }

    @Override
    public double getSensorValue() {
        return device.getState() ? 1 : 0;
    }

    public boolean getSensorValueAsBoolean() {
        return device.getState();
    }
}

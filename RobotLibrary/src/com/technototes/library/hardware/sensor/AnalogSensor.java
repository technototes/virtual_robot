package com.technototes.library.hardware.sensor;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class AnalogSensor extends Sensor<AnalogInput> {

    public AnalogSensor(AnalogInput d) {
        super(d);
    }

    @Override
    public double getSensorValue() {
        return device.getMaxVoltage();
    }

}

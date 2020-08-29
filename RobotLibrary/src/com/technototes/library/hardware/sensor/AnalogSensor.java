package com.technototes.library.hardware.sensor;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class AnalogSensor extends Sensor<AnalogInput> {

    public AnalogSensor(AnalogInput d) {
        super(d);
    }

    public AnalogSensor(String s) {
        super(s);
    }


    @Override
    public double getSensorValue() {
        return device.getMaxVoltage();
    }

}

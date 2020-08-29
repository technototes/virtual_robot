package com.technototes.library.hardware.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.technototes.library.hardware.HardwareDevice;
import com.technototes.library.hardware.PID;
import com.technototes.library.hardware.Sensored;
import com.technototes.library.hardware.sensor.encoder.Encoder;
import com.technototes.library.hardware.sensor.encoder.MotorEncoderSensor;
import com.technototes.library.util.PIDUtils;

public class EncodedMotor<T extends DcMotor> extends Motor<T> implements Sensored, PID {

    public double pid_p, pid_i, pid_d;
    public double threshold = 50;
    private Encoder encoder;

    public EncodedMotor(T d) {
        super(d);
        encoder = new MotorEncoderSensor(d);
    }

    public EncodedMotor(HardwareDevice<T> m) {
        super(m.getDevice());
    }

    public EncodedMotor(String s) {
        super(s);
    }

    @Override
    public EncodedMotor setInverted(boolean val) {
        device.setDirection(val ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        return this;
    }

    @Override
    public double getSensorValue() {
        return encoder.getSensorValue();
    }

    @Override
    public void setPIDValues(double p, double i, double d) {
        pid_p = p;
        pid_i = i;
        pid_d = d;
    }

    @Override
    public boolean setPositionPID(double val) {
        if (!isAtPosition(val)){
           setSpeed(PIDUtils.calculatePIDDouble(pid_p, pid_i, pid_d, getSensorValue(), val));
        } else {
            setSpeed(0);
            return true;
        }
        return false;
    }

    public boolean setPosition(double ticks) {
        return setPosition(ticks, 0.5);
    }

    public boolean setPosition(double ticks, double speed) {
        if (!isAtPosition(ticks)) {
            setSpeed(getSensorValue() < ticks ? speed : -speed);
        }else {
            setSpeed(0);
            return true;
        }
        return false;
    }

    public boolean isAtPosition(double ticks) {
        return Math.abs(ticks - getSensorValue()) < threshold;
    }

    public void resetEncoder() {
        encoder.zeroEncoder();
    }

}

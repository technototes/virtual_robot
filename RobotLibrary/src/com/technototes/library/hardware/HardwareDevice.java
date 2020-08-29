package com.technototes.library.hardware;

public abstract class HardwareDevice<T extends com.qualcomm.robotcore.hardware.HardwareDevice> {
    public T device;

    public HardwareDevice(T d) {
        device = d;
    }

    public HardwareDevice(HardwareDevice<T> d) {
        this(d.getDevice());
    }

    public T getDevice() {
        return device;
    }
}

package com.technototes.library.control.gamepad;

import com.technototes.library.command.Command;
import com.technototes.library.command.CommandScheduler;
import com.technototes.library.control.Trigger;

import java.util.function.BooleanSupplier;

public class ButtonGamepadComponent extends Trigger implements BooleanSupplier {
    protected BooleanSupplier booleanSupplier;
    protected boolean onoff, pastState1 = false, pastState2 = false, normalToggle = false, inverseToggle = true;

    public ButtonGamepadComponent(BooleanSupplier b) {
        booleanSupplier = b;
        onoff = b.getAsBoolean();
    }

    private boolean uponPress() {
        boolean b = false;
        if(getAsBoolean() && (pastState1 != getAsBoolean())){
            b=true;
            ///System.out.println("press");
        }
        pastState1 = getAsBoolean();
        return b;
    }

    private boolean uponRelease() {
        boolean b = false;
        if(!getAsBoolean() && (pastState2 != getAsBoolean())){
            b=true;
           // System.out.println("release");
        }
        pastState2 = getAsBoolean();
        return b;
    }

    @Override
    public ButtonGamepadComponent whenActivated(Command c) {
        CommandScheduler.getRunInstance().schedule(this::uponPress, c);
        return this;
    }

    @Override
    public ButtonGamepadComponent whenDeactivated(Command c) {
        CommandScheduler.getRunInstance().schedule(this::uponRelease, c);
        return this;
    }

    @Override
    public ButtonGamepadComponent whileActivated(Command c) {
        CommandScheduler.getRunInstance().schedule(() -> getAsBoolean(), c);
        return this;
    }

    @Override
    public ButtonGamepadComponent whileDeactivated(Command c) {
        CommandScheduler.getRunInstance().schedule(() -> !getAsBoolean(), c);
        return this;
    }

    @Override
    public ButtonGamepadComponent toggleWhenActivated(Command c) {
        CommandScheduler.getRunInstance().schedule(() -> (normalToggle = uponPress() ? !normalToggle : normalToggle) && uponPress(), c);
        return this;
    }

    @Override
    public ButtonGamepadComponent toggleWhenDeactivated(Command c) {
        CommandScheduler.getRunInstance().schedule(() -> (inverseToggle = uponRelease() ? !inverseToggle : inverseToggle) && uponRelease(), c);
        return this;
    }

    @Override
    public ButtonGamepadComponent toggleWhileActivated(Command c) {
        CommandScheduler.getRunInstance().schedule(() -> normalToggle = uponPress() ? !normalToggle : normalToggle, c);
        return this;
    }

    @Override
    public ButtonGamepadComponent toggleWhileDeactivated(Command c) {
        CommandScheduler.getRunInstance().schedule(() -> inverseToggle = uponRelease() ? !inverseToggle : inverseToggle, c);
        return this;
    }

    @Override
    public boolean getAsBoolean() {
        return booleanSupplier.getAsBoolean();
    }

}

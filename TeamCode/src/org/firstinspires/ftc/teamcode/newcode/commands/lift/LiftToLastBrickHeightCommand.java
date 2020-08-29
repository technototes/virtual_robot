package org.firstinspires.ftc.teamcode.newcode.commands.lift;

import com.technototes.library.command.Command;

import org.firstinspires.ftc.teamcode.newcode.subsystems.LiftSubsystem;

public class LiftToLastBrickHeightCommand extends Command {
    public LiftSubsystem subsystem;
    public int targetHeight;

    public LiftToLastBrickHeightCommand(LiftSubsystem s) {
        addRequirements(s);
        subsystem = s;
    }

    @Override
    public void init() {
        targetHeight = subsystem.lastPlacedBrickHeight+1;
    }

    @Override
    public boolean isFinished() {
        return subsystem.goToHeight(targetHeight);
    }

    @Override
    public void end() {
        subsystem.setHeightValue(targetHeight);
        subsystem.lastPlacedBrickHeight = targetHeight;
    }
}
package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class transfer extends CommandBase {
    boolean on = false;

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake intakeSubsystem;


    public transfer(Intake subsystem, boolean on) {
        intakeSubsystem = subsystem;
        this.on = on;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        if (on) {
            intakeSubsystem.startTransfer();
        } else {
            intakeSubsystem.stopTransfer();
        }
    }

    @Override
    public boolean isFinished() {
        if (on) {
            return (intakeSubsystem.getStopper() == 0.35);
        }
        else {
            return (intakeSubsystem.getStopper() == 0.48);
        }

    }
}

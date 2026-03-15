package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class slowTransfer extends CommandBase {
    boolean on = false;

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake intakeSubsystem;


    public slowTransfer(Intake subsystem, boolean on) {
        intakeSubsystem = subsystem;
        this.on = on;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        if (on) {
            intakeSubsystem.intake2On();
            intakeSubsystem.setStopper(0.35);
            intakeSubsystem.shooting = true;
            intakeSubsystem.active = false;
            intakeSubsystem.triggered = false;
        }
        else {
            intakeSubsystem.setStopper(0.5);
            intakeSubsystem.intakeOff();
            intakeSubsystem.active = true;
            intakeSubsystem.shooting = false;
        }
    }

    @Override
    public boolean isFinished() {
        if (on) {
            return (intakeSubsystem.getStopper() == 0.35);
        }
        else {
            return (intakeSubsystem.getStopper() == 0.5);
        }

    }
}
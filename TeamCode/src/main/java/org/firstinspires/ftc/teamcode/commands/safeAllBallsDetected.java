package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class safeAllBallsDetected extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake intakeSubsystem;


    public safeAllBallsDetected(Intake subsystem) {
        intakeSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.getCurrent() > 5
                && intakeSubsystem.isBallDetected01()
                && intakeSubsystem.isBallDetected02()
                && intakeSubsystem.isBallDetected03();
    }
}

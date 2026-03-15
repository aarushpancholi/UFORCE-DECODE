package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class shooterAutoShoot extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Shooter shooterSubsystem;


    public shooterAutoShoot(Shooter subsystem, Boolean autoShoot) {
        shooterSubsystem = subsystem;
        shooterSubsystem.setAutoShoot(autoShoot);
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
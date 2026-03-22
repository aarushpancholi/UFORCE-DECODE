package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class stationary extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Follower follower;


    public stationary(Follower subsystem) {
        follower = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements();
    }

    @Override
    public boolean isFinished() {
        return (follower.getVelocity().getMagnitude() < 0.2);
    }
}
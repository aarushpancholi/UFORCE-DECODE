package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.globals.Localization.getGoalDistance;
import static org.firstinspires.ftc.teamcode.subsystems.Shooter.getCoefficientsFromDistance;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class setShooterFromPose extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Shooter shooterSubsystem;
    private double target = 0;
    private double pos = 0.0;


    public setShooterFromPose(Shooter subsystem, Pose pose, String alliance) {
        shooterSubsystem = subsystem;
        double[] coefficients = getCoefficientsFromDistance(getGoalDistance(pose, alliance));
        this.target = coefficients[1];
        this.pos = coefficients[0];
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        //turn outtake on
        shooterSubsystem.setTargetEPT(target);
        shooterSubsystem.setHood(pos);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
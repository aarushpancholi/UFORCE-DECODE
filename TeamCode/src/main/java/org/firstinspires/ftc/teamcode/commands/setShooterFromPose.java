package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.globals.Localization.getGoalDistance;
import static org.firstinspires.ftc.teamcode.subsystems.Shooter.getCoefficientsFromDistance;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class setShooterFromPose extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Shooter shooterSubsystem;
    private final Pose targetPose;
    private final String alliance;
    private double target = 0;
    private double pos = 0.0;


    public setShooterFromPose(Shooter subsystem, Pose pose, String alliance) {
        shooterSubsystem = subsystem;
        this.targetPose = pose;
        this.alliance = alliance;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        double[] coefficients = getCoefficientsFromDistance(getGoalDistance(targetPose, alliance));
        this.target = coefficients[1];
        this.pos = coefficients[0];
        //turn outtake on
        shooterSubsystem.setTargetEPT(target);
        shooterSubsystem.setHood(pos);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

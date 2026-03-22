package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class setTurretPos extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Turret turretSubsystem;
    private Pose pose;


    public setTurretPos(Turret subsystem, Pose pos) {
        turretSubsystem = subsystem;
        this.pose = pos;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        turretSubsystem.setTurretPos(turretSubsystem.getTargetTicksFromPos(pose));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
package org.firstinspires.ftc.teamcode.main;

import static org.firstinspires.ftc.teamcode.globals.Localization.getGoalDistance;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.chosenAlliance;
import static org.firstinspires.ftc.teamcode.subsystems.Shooter.angleFromDistance;
import static org.firstinspires.ftc.teamcode.subsystems.Shooter.speedFromDistance;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commands.intakeOn1Command;
import org.firstinspires.ftc.teamcode.commands.isAimed;
import org.firstinspires.ftc.teamcode.commands.setShooter;
import org.firstinspires.ftc.teamcode.commands.transfer;
import org.firstinspires.ftc.teamcode.commands.turretAutoAim;
import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous(name = "Red Coop 9 Balls Auto")
public class RedNearSide9BallCoopAuto extends CommandOpMode {

    private Follower follower;
    private Shooter shooter;
    private Intake intake;
    private Turret turret;

    private TelemetryManager telemetryM;

    // Headings (radians) – keep same basis as your Red 18
    private static final double H0  = Math.toRadians(0);
    private static final double H45 = Math.toRadians(45);

    // --- Poses (based on Red 18, with the same “coop” tweaks seen in Blue) ---
    private final Pose startPose = new Pose(125.526, 115.883, H45);

    private final Pose p2End  = new Pose(100.587, 77.383);

    // +4 x like Blue coop did to its p3End
    private final Pose p3End  = new Pose(133.264, 77.821);

    private final Pose p4End  = new Pose(96.934, 98.054);

    // +5 y like Blue coop did to its p5End
    private final Pose p5End  = new Pose(98.886, 49.284 + 5.0);

    // Blue coop moved p6End up into the “ramp lane”; do the same here by matching that lane y
    // (this is the one place where the delta is not the same as Red 18, because Blue coop is aligning to the ramp line)
    private final Pose p6End  = new Pose(127.823, 56.361);

    // Ramp traverse + return points (same structure as Blue coop)
    private final Pose p7End   = new Pose(116.323, 54.361);
    private final Pose p8End   = new Pose(130.323, 63.361);
    private final Pose centerCP = new Pose(76, 64);

    // PathChains (same set as Blue coop, but built from Red coordinates)
    private PathChain path1, path2, path4, path2Shorter, path5;
    private PathChain redRampClear, path16RampExit, returnToShoot, lastPath, backClear, clear;

    private void buildPaths() {

        // path1: start -> p4 (shooting)
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(startPose.getX(), startPose.getY()),
                        new Pose(p4End.getX(), p4End.getY())
                ))
                .setConstantHeadingInterpolation(H45)
                .setBrakingStart(0.7)
                .build();

        // path2: p4 -> p2 -> p3 (pickup cycle)
        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(p4End.getX(), p4End.getY()),
                        new Pose(p2End.getX(), p2End.getY()),
                        new Pose(p3End.getX(), p3End.getY())
                ))
                .setLinearHeadingInterpolation(H45, H0, 0.3)
                .build();

        // path4: p3 -> p4 (return to shoot)
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(p3End.getX()+5, p3End.getY()-10),
                        new Pose(p4End.getX(), p4End.getY())
                ))
                .setBrakingStart(0.7)
                .setLinearHeadingInterpolation(H0, H45, 0.3)
                .build();

        // path2Shorter: same “end offset” method as Blue coop (+5, +4) to the p3 end
        path2Shorter = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(p4End.getX(), p4End.getY()),
                        new Pose(p2End.getX(), p2End.getY()),
                        new Pose(p3End.getX() + 5.0, p3End.getY() + 4.0)
                ))
                .setBrakingStart(0.7)
                .setLinearHeadingInterpolation(H45, H0, 0.3)
                .build();

        // path5: p4 -> p5 -> p6 (go to ramp-side pickup like Blue coop)
        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(p4End.getX(), p4End.getY()),
                        new Pose(p5End.getX(), p5End.getY()),
                        new Pose(p6End.getX(), p6End.getY())
                ))
                .setLinearHeadingInterpolation(H45, H0, 0.3)
                .build();

        // Ramp traverse pieces
        redRampClear = follower.pathBuilder()
                .addPath(new BezierLine(p6End, p7End))
                .setConstantHeadingInterpolation(H0)
                .build();

        path16RampExit = follower.pathBuilder()
                .addPath(new BezierLine(p7End, p8End))
                .setConstantHeadingInterpolation(H0)
                .build();

        // Return to shoot: p8 -> center -> p4
        returnToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        p6End,
                        centerCP,
                        p4End
                ))
                .setLinearHeadingInterpolation(H0, H45, 0.3)
                .setBrakingStart(0.7)
                .build();

        // Final park / clear: same “+20 x” method as Blue coop (heading held)
        lastPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        p4End,
                        new Pose(p3End.getX() - 20.0, p3End.getY())
                ))
                .setConstantHeadingInterpolation(H0)
                .setBrakingStart(0.7)
                .build();

        backClear = follower.pathBuilder()
                .addPath(new BezierLine(
                        p3End,
                        new Pose(p3End.getX()-35, p3End.getY())
                ))
                .setConstantHeadingInterpolation(H0)
                .build();

        clear = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(p3End.getX()-35, p3End.getY()),
                        new Pose(p3End.getX()+5, p3End.getY()-4.3)
                ))
                .setConstantHeadingInterpolation(H0)
                .build();
    }

    @Override
    public void initialize() {
        super.reset();

        RobotConstants.chosenAlliance = "RED";
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        intake = new Intake(hardwareMap, telemetryM);
        shooter = new Shooter(hardwareMap, telemetryM);
        turret = new Turret(hardwareMap, telemetryM);

        intake.setStopper(0.45);
        turret.resetTurretEncoder();
        turret.isAutoCode = true;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();
        Localization.init(follower, telemetryM);

        SequentialCommandGroup shooterSequence = new SequentialCommandGroup(
                new WaitCommand(300),
                new isAimed(turret).withTimeout(400),
                new transfer(intake, true).alongWith(new InstantCommand(() -> intake.intake2On())),
                new WaitCommand(600)
        );

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                new turretAutoAim(turret, true),

                // 1) Go shoot from p4
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, path1),
                        new setShooter(shooter, (int) speedFromDistance(getGoalDistance(p4End, chosenAlliance)), angleFromDistance(getGoalDistance(p4End, chosenAlliance))),
                        new intakeOn1Command(intake)
                ),
                new WaitCommand(750),
                shooterSequence,

//                new WaitCommand()

                // 2) Go to p6 (pickup), then traverse ramp, wait, return to p4 and shoot
                new ParallelCommandGroup(
                        new transfer(intake, false).alongWith(new InstantCommand(() -> intake.intake2Off())),
                        new FollowPathCommand(follower, path5)
                ),
//                new FollowPathCommand(follower, redRampClear),
//                new FollowPathCommand(follower, path16RampExit).withTimeout(750),
//                new WaitCommand(1500),
                new FollowPathCommand(follower, returnToShoot),
                shooterSequence,

                // 3) Shorter cycle (path2Shorter) -> return -> shoot
                new ParallelCommandGroup(
                        new transfer(intake, false).alongWith(new InstantCommand(() -> intake.intake2Off())),
                        new FollowPathCommand(follower, path2).setGlobalMaxPower(1)
                ),
                new FollowPathCommand(follower, backClear),
                new FollowPathCommand(follower, clear),
                new WaitCommand(1500),
                new FollowPathCommand(follower, path4),
                shooterSequence,
//
//                // 4) Full cycle (path2) -> (implicit return via next path4 if you want it) then park
//                new ParallelCommandGroup(
//                        new transfer(intake, false).alongWith(new InstantCommand(() -> intake.intake2Off())),
//                        new FollowPathCommand(follower, path2Shorter).setGlobalMaxPower(1)
//                ),
                new WaitCommand(1500),
                new FollowPathCommand(follower, lastPath)
        );

        schedule(autonomousSequence);
    }

    @Override
    public void run() {
        super.run();

        Localization.update();
        turret.periodic();
        shooter.periodic();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("distance", getGoalDistance(follower.getPose(), chosenAlliance));
        telemetry.update();
    }

    @Override
    public void end() {
        RobotConstants.savedPose = follower.getPose();
    }
}
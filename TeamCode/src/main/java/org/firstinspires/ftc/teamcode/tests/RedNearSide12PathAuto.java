package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.globals.Localization.getGoalDistance;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.chosenAlliance;

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
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;

import org.firstinspires.ftc.teamcode.commands.allBallsDetected;
import org.firstinspires.ftc.teamcode.commands.transfer;
import org.firstinspires.ftc.teamcode.commands.turretAutoAim;
import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous(name = "Red Near Side 12 Path Auto")
public class RedNearSide12PathAuto extends CommandOpMode {

    private Follower follower;
    private Shooter shooter;
    private Intake intake;
    private Turret turret;

    private TelemetryManager telemetryM;

    private static final double H0 = Math.toRadians(0);
    private static final double H45 = Math.toRadians(45);

    // Path points (deduped for repeated/similar points)
    private final Pose startPose = new Pose(102.018, 136.72, H0); // near side new start pos
    private final Pose shootPose = new Pose(87.139, 82.928, H0);
    private final Pose p2End = new Pose(128.319, 80.593, H0);
    private final Pose cycleReturnPose = new Pose(82.838, 71.880, H0);
    private final Pose p4Control = new Pose(96.870, 58.398);
    private final Pose p4End = new Pose(129.42, 61.39, H0);
    private final Pose shuttlePose = new Pose(131.806, 60.429, H45); // gate ramp collect direct pose
    private final Pose p10End = new Pose(95.016, 35.831, H0);
    private final Pose p11End = new Pose(134.261, 38.246, H0);
    private final Pose p12End = new Pose(86.208, 19.095, H0);

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11, path12;

    private void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(H0, H0)
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, p2End))
                .setConstantHeadingInterpolation(H0)
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(p2End, cycleReturnPose))
                .setConstantHeadingInterpolation(H0)
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(cycleReturnPose, p4Control, p4End))
                .setConstantHeadingInterpolation(H0)
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(p4End, cycleReturnPose))
                .setConstantHeadingInterpolation(H0)
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierLine(cycleReturnPose, shuttlePose))
                .setLinearHeadingInterpolation(H0, H45)
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierLine(shuttlePose, cycleReturnPose))
                .setLinearHeadingInterpolation(H45, H0)
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierLine(cycleReturnPose, shuttlePose))
                .setLinearHeadingInterpolation(H0, H45)
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(shuttlePose, cycleReturnPose))
                .setLinearHeadingInterpolation(H45, H0)
                .build();

        path10 = follower.pathBuilder()
                .addPath(new BezierLine(cycleReturnPose, p10End))
                .setConstantHeadingInterpolation(H0)
                .build();

        path11 = follower.pathBuilder()
                .addPath(new BezierLine(p10End, p11End))
                .setConstantHeadingInterpolation(H0)
                .build();

        path12 = follower.pathBuilder()
                .addPath(new BezierLine(p11End, p12End))
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
        shooter.setAutoShoot(true);

        intake.setStopper(0.5);
        turret.resetTurretEncoder();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        super.register(turret);
        super.register(shooter);

        buildPaths();
        Localization.init(follower, telemetryM);

        SequentialCommandGroup shooterSequence = new SequentialCommandGroup(
                new WaitCommand(200),
                new transfer(intake, true)
                        .alongWith(new InstantCommand(() -> intake.intake2On())),
                new WaitCommand(400)
        );

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                new turretAutoAim(turret, true),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, path1),
                        new SequentialCommandGroup(
                                new WaitCommand(1500),
                                new transfer(intake, true)
                                        .alongWith(new InstantCommand(() -> intake.intake2On())),
                                new WaitCommand(400)
                        )
                ),
                new WaitCommand(500),
                new ParallelCommandGroup(
                        new transfer(intake, false)
                                .alongWith(new InstantCommand(() -> intake.intake2Off())),
                        new FollowPathCommand(follower, path2).setGlobalMaxPower(1)
                ),
                new FollowPathCommand(follower, path3),
                shooterSequence,
                new ParallelCommandGroup(
                        new transfer(intake, false)
                                .alongWith(new InstantCommand(() -> intake.intake2Off())),
                        new FollowPathCommand(follower, path4)
                ),
                new FollowPathCommand(follower, path5),
                shooterSequence,
                new ParallelCommandGroup(
                        new transfer(intake, false)
                                .alongWith(new InstantCommand(() -> intake.intake2Off())),
                        new FollowPathCommand(follower, path6)
                ),
                new FollowPathCommand(follower, path7),
                shooterSequence,
                new ParallelCommandGroup(
                        new transfer(intake, false)
                                .alongWith(new InstantCommand(() -> intake.intake2Off())),
                        new FollowPathCommand(follower, path8)
                ),
                new TurnToCommand(follower, H45).withTimeout(100),
                new ParallelRaceGroup(
                        new allBallsDetected(intake),
                        new WaitCommand(1500)
                ),
                new FollowPathCommand(follower, path9),
                shooterSequence,
                new ParallelCommandGroup(
                        new transfer(intake, false)
                                .alongWith(new InstantCommand(() -> intake.intake2Off())),
                        new FollowPathCommand(follower, path10)
                ),
                new FollowPathCommand(follower, path11),
                shooterSequence,
                new FollowPathCommand(follower, path12)
        );

        schedule(autonomousSequence);
    }

    @Override
    public void run() {
        super.run();

        Localization.update();

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

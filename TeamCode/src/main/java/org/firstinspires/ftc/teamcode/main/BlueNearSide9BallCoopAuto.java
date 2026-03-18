package org.firstinspires.ftc.teamcode.main;

import static org.firstinspires.ftc.teamcode.globals.Localization.getGoalDistance;
import static org.firstinspires.ftc.teamcode.globals.Localization.getRedDistance;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.chosenAlliance;
import static org.firstinspires.ftc.teamcode.subsystems.Shooter.angleFromDistance;
import static org.firstinspires.ftc.teamcode.subsystems.Shooter.speedFromDistance;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
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

@Autonomous(name = "Blue Coop 9 Balls Auto")
public class BlueNearSide9BallCoopAuto extends CommandOpMode {

    private Follower follower;
    private Shooter shooter;
    private Intake intake;
    private Turret turret;

    private TelemetryManager telemetryM;

    // Headings (radians)
    private static final double H180 = Math.toRadians(180);
    private static final double H160 = Math.toRadians(216);
    private static final double H155 = Math.toRadians(148);
    private static final double H145 = Math.toRadians(145);
    private static final double H135 = Math.toRadians(135);


    // --- Poses (from your provided Paths array) ---


    private final Pose startPose = new Pose(129.526, 127.883, Math.toRadians(45)).mirror();

    private final Pose p1End  = new Pose(84.931, 89.214).mirror();
    private final Pose p2End  = new Pose(102.087, 83.383).mirror();
    private final Pose rampCollectCP   = new Pose(104.703, 53.168).mirror();
    private final Pose rampCollectEnd  = new Pose(132.8, 65.5).mirror();

    private final Pose p3End  = new Pose(130.764, 83.921).mirror();
    private final Pose p4End  = new Pose(100, 100).mirror();
    private final Pose p5End  = new Pose(94.386, 64.284).mirror();

    private final Pose p6End  = new Pose(126.323, 64.361).mirror();
    private final Pose p7End = new Pose(23.677, 64.361);
    private final Pose p8End = new Pose(12.677, 73.361);
    private final Pose centerCP = new Pose(64, 64);


    // PathChains
    private PathChain path1, path2, path4, path2Shorter, path5, path7,
            blueRampClear, path16RampExit, lastPath, rampCollectPath, secondLastPath, thirdLastPath;

    private void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(startPose.getX(), startPose.getY()),
                        new Pose(p4End.getX(), p4End.getY())
                ))
                .setConstantHeadingInterpolation(H135)
                .setBrakingStart(0.7)
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(p4End.getX(), p4End.getY()),
                        new Pose(p2End.getX(), p2End.getY()),
                        new Pose(p3End.getX(), p3End.getY())
                ))
                .setLinearHeadingInterpolation(H135, H180, 0.3)
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(p3End.getX(), p3End.getY()),
                        new Pose(p4End.getX(), p4End.getY())
                ))
                .setBrakingStart(0.7)
                .setLinearHeadingInterpolation(H180, H135, 0.3)
                .build();

        rampCollectPath = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(p4End.getX(), p4End.getY()),
                        new Pose(rampCollectCP.getX(), rampCollectCP.getY()),
                        new Pose(rampCollectEnd.getX(), rampCollectEnd.getY())
                ))
                .setLinearHeadingInterpolation(H135, H145, 0.3)
                .build();


        path2Shorter =
                follower.pathBuilder()
                        .addPath(new BezierCurve(
                                new Pose(p4End.getX(), p4End.getY()),
                                new Pose(p2End.getX(), p2End.getY()),
                                new Pose(p3End.getX() + 5, p3End.getY() + 4)
                        ))
                        .setBrakingStart(0.7)
                        .setLinearHeadingInterpolation(H135, H180, 0.3)
                        .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(p4End.getX(), p4End.getY()),
                        new Pose(p5End.getX(), p5End.getY()),
                        new Pose(p6End.getX(), p6End.getY())
                ))
                .setLinearHeadingInterpolation(H135, H180, 0.3)
                .build();

        blueRampClear = follower.pathBuilder()
                .addPath(new BezierLine(
                        p6End,
                        p7End
                ))
                .setConstantHeadingInterpolation(H180)
                .build();

        path16RampExit = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                p7End, p8End
                        )
                )
                .setConstantHeadingInterpolation(H180)
                .build();


        path7 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        p8End,
                        centerCP,
                        p4End
                ))
                .setLinearHeadingInterpolation(H180, H135, 0.3)
                .setBrakingStart(0.7)
                .build();

        secondLastPath = follower.pathBuilder()
                .addPath(new BezierCurve(
                        p3End,
                        new Pose(p3End.getX() + 35, p3End.getY()),
                        p4End
                ))
                .setLinearHeadingInterpolation(H155, H135)
                .setBrakingStart(0.7)
                .build();

        lastPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        p4End,
                        new Pose(p3End.getX() + 20, p3End.getY())
                ))
                .setLinearHeadingInterpolation(H135, H180)
                .setBrakingStart(0.7)
                .build();



    }

    @Override
    public void initialize() {
        super.reset();

        RobotConstants.chosenAlliance = "BLUE";
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
                new WaitCommand(200),
                new isAimed(turret).withTimeout(400),
                new transfer(intake, true)
                        .alongWith(new InstantCommand(() -> intake.intake2On())),
                new WaitCommand(700)
        );

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                new turretAutoAim(turret, true),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, path1),
                        new setShooter(shooter, 1185, 0.6),
                        new intakeOn1Command(intake)
                ),
                shooterSequence,
                new ParallelCommandGroup(
                        new transfer(intake, false)
                                .alongWith(new InstantCommand(() -> intake.intake2Off())),
                        new FollowPathCommand(follower, path5)
                ),
                new FollowPathCommand(follower, blueRampClear),
                new FollowPathCommand(follower, path16RampExit).withTimeout(750),
                new WaitCommand(1500),
                new FollowPathCommand(follower, path7),
                shooterSequence,
                new ParallelCommandGroup(
                        new transfer(intake, false)
                                .alongWith(new InstantCommand(() -> intake.intake2Off())),
                        new FollowPathCommand(follower, path2Shorter).setGlobalMaxPower(1)
                ),
                new FollowPathCommand(follower, path4),
                shooterSequence,
                new ParallelCommandGroup(
                        new transfer(intake, false)
                                .alongWith(new InstantCommand(() -> intake.intake2Off())),
                        new FollowPathCommand(follower, path2).setGlobalMaxPower(1)
                ),
                new WaitCommand(700),
                new FollowPathCommand(follower, secondLastPath),
                new ParallelCommandGroup(
                        new transfer(intake, false)
                                .alongWith(new InstantCommand(() -> intake.intake2Off())),
                        new FollowPathCommand(follower, rampCollectPath)
                ),
                new TurnToCommand(follower, H155).withTimeout(100),
                new ParallelRaceGroup(
                        new allBallsDetected(intake),
                        new WaitCommand(1500)

                ),
                new FollowPathCommand(follower, secondLastPath),
                shooterSequence,
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

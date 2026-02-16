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

@Autonomous(name = "Blue 18 Balls Auto")
public class BlueNearSide18BallsAuto extends CommandOpMode {

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
    private final Pose p2End  = new Pose(102.087, 86.583).mirror();
    private final Pose p3End  = new Pose(126.764, 85.321).mirror();
    private final Pose p4End  = new Pose(100, 100).mirror();
    private final Pose p5End  = new Pose(94.386, 59.284).mirror();

    private final Pose p6CP   = new Pose(124.085, 58.610).mirror();
    private final Pose p6End  = new Pose(126.323, 59.361).mirror();

    private final Pose p8CP   = new Pose(104.703, 53.168).mirror();
    private final Pose p8End  = new Pose(132.8, 65.5).mirror();

    private final Pose p9End = new Pose(85.626, 37.187).mirror();
    private final Pose p10End = new Pose(127.716, 38.051).mirror();
    private final Pose p11End = new Pose(76.578, 20.858).mirror();
    private final Pose p12End = new Pose(76.707, 27.152).mirror();


    // PathChains
    private PathChain path1, path2, path3, path4, path5, path6, path7,
            path8, path9, path10, path11, path12, path13, path14, path15, newPath14, newPath15;

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
                .setLinearHeadingInterpolation(H135, H180, 0.2)
                .build();
//
//        path3 = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        new Pose(p2End.getX(), p2End.getY()),
//                        new Pose(p3End.getX(), p3End.getY())
//                ))
//                .setConstantHeadingInterpolation(H0)
//                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(p3End.getX(), p3End.getY()),
                        new Pose(p4End.getX(), p4End.getY())
                ))
                .setBrakingStart(0.7)
                .setLinearHeadingInterpolation(H180, H135, 0.2)
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(p4End.getX(), p4End.getY()),
                        new Pose(p5End.getX(), p5End.getY()),
                        new Pose(p6End.getX(), p6End.getY())
                ))
                .setLinearHeadingInterpolation(H135, H180, 0.2)
                .build();

//        path6 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        new Pose(p5End.getX(), p5End.getY()),
//
//                ))
//                .setLinearHeadingInterpolation(H0)
//                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(p6End.getX(), p6End.getY()),
                        new Pose(p4End.getX(), p4End.getY())
                ))
                .setLinearHeadingInterpolation(H160, H135, 0.2)
                .setBrakingStart(0.7)
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(p4End.getX(), p4End.getY()),
                        new Pose(p8CP.getX(), p8CP.getY()),
                        new Pose(p8End.getX(), p8End.getY())
                ))
                .setLinearHeadingInterpolation(H135, H145, 0.2)
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(p8End.getX(), p8End.getY()),
                        new Pose(p4End.getX(), p4End.getY())
                ))
                .setLinearHeadingInterpolation(H155, H135)
                .setBrakingStart(0.7)
                .build();

        path10 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(p4End.getX(), p4End.getY()),
                        new Pose(p8CP.getX(), p8CP.getY()),
                        new Pose(p8End.getX(), p8End.getY())
                ))
                .setLinearHeadingInterpolation(H135, H145, 0.2)
                .build();

        path11 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(p8End.getX(), p8End.getY()),
                        new Pose(p4End.getX(), p4End.getY())
                ))
                .setLinearHeadingInterpolation(H155, H135, 0.2)
                .setBrakingStart(0.7)
                .build();

        path12 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(p4End.getX(), p4End.getY()),
                        new Pose(p9End.getX(), p9End.getY()),
                        new Pose(p10End.getX(), p10End.getY())
                ))
                .setLinearHeadingInterpolation(H135, H180, 0.2)
                .build();

//        path13 = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        new Pose(p9End.getX(), p9End.getY()),
//                        new Pose(p10End.getX(), p10End.getY())
//                ))
//                .setConstantHeadingInterpolation(H0)
//                .build();

        path14 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(p10End.getX(), p10End.getY()),
                        new Pose(p11End.getX(), p11End.getY())
                ))
                .setLinearHeadingInterpolation(H180, Math.toRadians(125))
                .build();

        path15 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(p11End.getX(), p11End.getY()),
                        new Pose(p12End.getX(), p12End.getY())
                ))
                .setConstantHeadingInterpolation(Math.toRadians(125))
                .build();

        newPath14 = follower.pathBuilder()
                .addPath(new BezierLine(p10End, p4End))
                .setLinearHeadingInterpolation(H180, H135)
                .build();

        newPath15 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(p4End.getX(), p4End.getY()),
                        new Pose(p4End.getX(), 72)
                ))
                .setConstantHeadingInterpolation(H135)
                .build();
    }

    @Override
    public void initialize() {
        super.reset();

        RobotConstants.chosenAlliance = "BLUE";
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        intake = new Intake(hardwareMap, telemetryM);
        shooter = new Shooter(hardwareMap, telemetryM, true);
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
                new isAimed(turret).withTimeout(500),
                new transfer(intake, true)
                        .alongWith(new InstantCommand(() -> intake.intake2On())),
                new WaitCommand(800)
        );

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                new turretAutoAim(turret, true),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, path1),
                        new setShooter(shooter, 1200, 0.6),
                        new intakeOn1Command(intake)
                ),
                shooterSequence,
                new ParallelCommandGroup(
//                        new setShooter(shooter, (int) speedFromDistance(getGoalDistance(p4End, chosenAlliance)), angleFromDistance(getGoalDistance(p4End, chosenAlliance))),
                        new transfer(intake, false)
                                .alongWith(new InstantCommand(() -> intake.intake2Off())),
                        new FollowPathCommand(follower, path2).setGlobalMaxPower(1)
                ),
//                new FollowPathCommand(follower, path3),
                new FollowPathCommand(follower, path4),
                shooterSequence,
                new ParallelCommandGroup(
                        new transfer(intake, false)
                                .alongWith(new InstantCommand(() -> intake.intake2Off())),
                        new FollowPathCommand(follower, path5)
                ),
                new FollowPathCommand(follower, path7),
                shooterSequence,
                new ParallelCommandGroup(
                        new transfer(intake, false)
                                .alongWith(new InstantCommand(() -> intake.intake2Off())),
                        new FollowPathCommand(follower, path8)
                ),
                new TurnToCommand(follower, H155).withTimeout(200),
                new ParallelRaceGroup(
                        new allBallsDetected(intake),
                        new WaitCommand(1500)

                ),
                new FollowPathCommand(follower, path9),
                shooterSequence,

                new ParallelCommandGroup(
                        new transfer(intake, false)
                                .alongWith(new InstantCommand(() -> intake.intake2Off())),
                        new FollowPathCommand(follower, path12)
                ),
                new FollowPathCommand(follower, newPath14),
                shooterSequence,
                new FollowPathCommand(follower, newPath15)

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

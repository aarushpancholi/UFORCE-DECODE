package org.firstinspires.ftc.teamcode.tests;

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
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;

import org.firstinspires.ftc.teamcode.commands.allBallsDetected;
import org.firstinspires.ftc.teamcode.commands.intakeOn1Command;
import org.firstinspires.ftc.teamcode.commands.intakeOn2Command;
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

@Autonomous(name = "Auto18Points")
public class Auto18Points extends CommandOpMode {

    private Follower follower;
    private Shooter shooter;
    private Intake intake;
    private Turret turret;

    private TelemetryManager telemetryM;

    // Headings (radians)
    private static final double H0  = Math.toRadians(0);
    private static final double H20 = Math.toRadians(20);
    private static final double H25 = Math.toRadians(32);
    private static final double H35 = Math.toRadians(35);
    private static final double H45 = Math.toRadians(45);

    // --- Poses (from your provided Paths array) ---
    private final Pose startPose = new Pose(125.526, 115.883, H45);

    private final Pose p1End  = new Pose(87.431, 89.214);
    private final Pose p2End  = new Pose(104.587, 83.383);
    private final Pose p3End  = new Pose(130.264, 83.921);
    private final Pose p4End  = new Pose(81.889, 81.420);
    private final Pose p5End  = new Pose(104.886, 59.284);

    private final Pose p6CP   = new Pose(126.585, 56.610);
    private final Pose p6End  = new Pose(129.823, 56.361);

    private final Pose p8CP   = new Pose(117.203, 53.168);
    private final Pose p8End  = new Pose(134, 56.7);

    private final Pose p9End = new Pose(105.126, 35.187);
    private final Pose p10End = new Pose(135.216, 35.051);
    private final Pose p11End = new Pose(84.078, 20.858);
    private final Pose p12End = new Pose(84.207, 27.152);


    // PathChains
    private PathChain path1, path2, path3, path4, path5, path6, path7,
            path8, path9, path10, path11, path12, path13, path14, path15;

    private void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(startPose.getX(), startPose.getY()),
                        new Pose(p1End.getX(), p1End.getY())
                ))
                .setConstantHeadingInterpolation(H45)
                .setBrakingStart(0.7)
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(p1End.getX(), p1End.getY()),
                        new Pose(p2End.getX(), p2End.getY()),
                        new Pose(p3End.getX(), p3End.getY())
                ))
                .setLinearHeadingInterpolation(H45, H0, 0.3)
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
                .setLinearHeadingInterpolation(H0, H45, 0.3)
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(p4End.getX(), p4End.getY()),
                        new Pose(p5End.getX(), p5End.getY()),
                        new Pose(p6End.getX(), p6End.getY())
                ))
                .setLinearHeadingInterpolation(H45, H0, 0.3)
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
                .setLinearHeadingInterpolation(H20, H45, 0.3)
                .setBrakingStart(0.7)
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(p4End.getX(), p4End.getY()),
                        new Pose(p8CP.getX(), p8CP.getY()),
                        new Pose(p8End.getX(), p8End.getY())
                ))
                .setLinearHeadingInterpolation(H45, H35, 0.3)
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(p8End.getX(), p8End.getY()),
                        new Pose(p4End.getX(), p4End.getY())
                ))
                .setLinearHeadingInterpolation(H25, H45)
                .setBrakingStart(0.7)
                .build();

        path10 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(p4End.getX(), p4End.getY()),
                        new Pose(p8CP.getX(), p8CP.getY()),
                        new Pose(p8End.getX(), p8End.getY())
                ))
                .setLinearHeadingInterpolation(H45, H35, 0.3)
                .build();

        path11 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(p8End.getX(), p8End.getY()),
                        new Pose(p4End.getX(), p4End.getY())
                ))
                .setLinearHeadingInterpolation(H25, H45, 0.3)
                .setBrakingStart(0.7)
                .build();

        path12 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(p4End.getX(), p4End.getY()),
                        new Pose(p9End.getX(), p9End.getY()),
                        new Pose(p10End.getX(), p10End.getY())
                ))
                .setLinearHeadingInterpolation(H45, H0, 0.3)
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
                .setLinearHeadingInterpolation(H0, Math.toRadians(55))
                .build();

        path15 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(p11End.getX(), p11End.getY()),
                        new Pose(p12End.getX(), p12End.getY())
                ))
                .setConstantHeadingInterpolation(Math.toRadians(55))
                .build();
    }

    @Override
    public void initialize() {
        super.reset();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        intake = new Intake(hardwareMap, telemetryM);
        shooter = new Shooter(hardwareMap, telemetryM);
        turret = new Turret(hardwareMap, telemetryM);

        intake.setStopper(0.45);
        turret.resetTurretEncoder();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();
        Localization.init(follower, telemetryM);

        SequentialCommandGroup shooterSequence = new SequentialCommandGroup(
                new WaitCommand(200),
                new isAimed(turret).withTimeout(400),
                new transfer(intake, true)
                        .alongWith(new InstantCommand(() -> intake.intake2On())),
                new WaitCommand(450)
        );

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                new turretAutoAim(turret, true),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, path1),
                        new setShooter(shooter, 1215, angleFromDistance(getGoalDistance(p1End, chosenAlliance))),
                        new intakeOn1Command(intake)
                ),
                shooterSequence,
                new ParallelCommandGroup(
                        new setShooter(shooter, 1365, angleFromDistance(getGoalDistance(p4End, chosenAlliance))),
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
                        new setShooter(shooter, 1355, angleFromDistance(getGoalDistance(p4End, chosenAlliance))),
                        new transfer(intake, false)
                                .alongWith(new InstantCommand(() -> intake.intake2Off())),
                        new FollowPathCommand(follower, path8)
                ),
                new TurnToCommand(follower, H25).withTimeout(100),
                new ParallelRaceGroup(
                        new allBallsDetected(intake),
                        new WaitCommand(750)

                ),
                new FollowPathCommand(follower, path9),
                shooterSequence,
                new ParallelCommandGroup(
                        new transfer(intake, false)
                                .alongWith(new InstantCommand(() -> intake.intake2Off())),
                        new FollowPathCommand(follower, path10)
                ),
                new TurnToCommand(follower, H25).withTimeout(100),
                new ParallelRaceGroup(
                        new allBallsDetected(intake),
                        new WaitCommand(750)

                ),
                new FollowPathCommand(follower, path11),
                shooterSequence,
                new ParallelCommandGroup(
                        new setShooter(shooter, 1630, angleFromDistance(getGoalDistance(p11End, chosenAlliance))),
                        new transfer(intake, false)
                                .alongWith(new InstantCommand(() -> intake.intake2Off())),
                        new FollowPathCommand(follower, path12)
                ),
                new FollowPathCommand(follower, path14),
                shooterSequence,
                new FollowPathCommand(follower, path15)

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

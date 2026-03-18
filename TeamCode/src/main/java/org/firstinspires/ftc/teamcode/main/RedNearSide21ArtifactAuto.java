package org.firstinspires.ftc.teamcode.main;

import static org.firstinspires.ftc.teamcode.globals.RobotConstants.chosenAlliance;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commands.allBallsDetected;
import org.firstinspires.ftc.teamcode.commands.allBallsGone;
import org.firstinspires.ftc.teamcode.commands.intakeOn1Command;
import org.firstinspires.ftc.teamcode.commands.isAimed;
import org.firstinspires.ftc.teamcode.commands.safeAllBallsDetected;
import org.firstinspires.ftc.teamcode.commands.setShooterFromPose;
import org.firstinspires.ftc.teamcode.commands.shooterAtSpeed;
import org.firstinspires.ftc.teamcode.commands.slowTransfer;
import org.firstinspires.ftc.teamcode.commands.transfer;
import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous(name = "Red Near Side 21 Path Auto")
public class RedNearSide21ArtifactAuto extends CommandOpMode {

    private Follower follower;
    private TelemetryManager telemetryM;
    private Intake intake;
    private Turret turret;
    private Shooter shooter;

    private static final double H0 = Math.toRadians(0);
    private static final double H30 = Math.toRadians(30);
    private static final double H60 = Math.toRadians(60);
    private static final double H80 = Math.toRadians(80);
    private static final double H_NEG20 = Math.toRadians(-20);

    private final Pose startPose = new Pose(106.018, 136.72, H0);
    private final Pose p1End = new Pose(93, 90);
    private final Pose p2End = new Pose(117.5, 83.321);
    private final Pose p3End = new Pose(97.440, 83.546);
    private final Pose p4Control = new Pose(97.000, 56.208);
    private final Pose p4End = new Pose(121, 57);
    private final Pose p5End = new Pose(89.598, 73.020);
    private final Pose p6Control = new Pose(116.651, 57.314);
    private final Pose directRampPrePose = new Pose(126, 61.5);
    private final Pose rampCollectPose = new Pose(131, 60.429, H60);
    private final Pose rampFinishPose = new Pose(135, 56.429);
    private final Pose rampBackPose = new Pose(135.161, 51.20);
    private final Pose directRampCollectPose = new Pose(131.806,61.5);
    private final Pose farSideShootPose = new Pose(86.885, 16.606);
    private final Pose finalMarkStart = new Pose(82.48, 31);
    private final Pose finalMarkEnd = new Pose(121, 31);
    private final Pose humanPlayerPose = new Pose(134, 6);
    private int loopCounter;
    private ElapsedTime elapsedtime;

    private PathChain path1, path2, path3, path4, path5, rampCollectPath, rampReturnPath, rampFinishCollectPath, rampBackCollectPath, directRampCollectPath1, directRampCollectPath2, directRampReturnPath, rampToFarSidePath, lastMarkCollect, lastMarkToShoot, humanPlayerCollectPath, humanPlayerToShoot, humanPlayerCollectPathFinish;

    private void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, p1End))
                .setConstantHeadingInterpolation(H0)
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(p1End, p2End))
                .setConstantHeadingInterpolation(H0)
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(p2End, p3End))
                .setConstantHeadingInterpolation(H0)
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(p3End, p4Control, p4End))
                .setConstantHeadingInterpolation(H0)
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(p4End, p5End))
                .setLinearHeadingInterpolation(H0, H0)
                .build();

        directRampCollectPath1 = follower.pathBuilder()
                .addPath(new BezierLine(p5End, directRampPrePose))
                .setLinearHeadingInterpolation(H0, H30, 0.3)
                .setBrakingStart(0.5)
                .build();

        directRampCollectPath2 = follower.pathBuilder()
                .addPath(new BezierLine(directRampPrePose, directRampCollectPose))
                .setConstantHeadingInterpolation(H30)
                .setBrakingStart(0.5)
                .build();

        directRampReturnPath = follower.pathBuilder()
                .addPath(new BezierCurve(directRampCollectPose, p6Control, p5End))
                .setLinearHeadingInterpolation(H30, H0)
                .build();

        rampCollectPath = follower.pathBuilder()
                .addPath(new BezierCurve(p5End, p6Control, rampCollectPose))
                .setLinearHeadingInterpolation(H_NEG20, H60)
                .build();

        rampBackCollectPath = follower.pathBuilder()
                .addPath(new BezierLine(rampCollectPose, rampBackPose))
                .setLinearHeadingInterpolation(H60, H80)
                .build();

        rampFinishCollectPath = follower.pathBuilder()
                .addPath(new BezierLine(rampBackPose, rampFinishPose))
                .setConstantHeadingInterpolation(H80)
                .build();

        rampReturnPath = follower.pathBuilder()
                .addPath(new BezierCurve(rampFinishPose, p6Control, p5End))
                .setLinearHeadingInterpolation(H80, H_NEG20)
                .build();

        rampToFarSidePath = follower.pathBuilder()
                .addPath(new BezierCurve(directRampCollectPose, p6Control, farSideShootPose))
                .setLinearHeadingInterpolation(H30, H0)
                .build();

        lastMarkCollect = follower.pathBuilder()
                .addPath(new BezierCurve(p5End, finalMarkStart, finalMarkEnd))
                .setConstantHeadingInterpolation(H0)
                .build();

        lastMarkToShoot = follower.pathBuilder()
                .addPath(new BezierLine(finalMarkEnd, farSideShootPose))
                .setConstantHeadingInterpolation(H0)
                .build();

        humanPlayerCollectPath = follower.pathBuilder()
                .addPath(new BezierLine(farSideShootPose, humanPlayerPose))
                .setConstantHeadingInterpolation(H0)
                .build();

        humanPlayerCollectPathFinish = follower.pathBuilder()
                .addPath(new BezierLine(humanPlayerPose, new Pose(humanPlayerPose.getX(), humanPlayerPose.getY() + 3)))
                .setConstantHeadingInterpolation(H0)
                .build();

        humanPlayerToShoot = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(humanPlayerPose.getX(), humanPlayerPose.getY()), farSideShootPose))
                .setConstantHeadingInterpolation(H0)
                .build();
    }

    @Override
    public void initialize() {
        super.reset();
        elapsedtime = new ElapsedTime();
        elapsedtime.reset();

        chosenAlliance = "RED";
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        intake = new Intake(hardwareMap, telemetryM);
        shooter = new Shooter(hardwareMap, telemetryM);
        turret = new Turret(hardwareMap, telemetryM);

        intake.setAutoEnabled(true);
        shooter.setAutoShoot(true);
        turret.autoAimEnabled = true;

        intake.setStopper(0.45);
        turret.resetTurretEncoder();

        super.register(intake);
        super.register(shooter);
        super.register(turret);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();
        Localization.init(follower, telemetryM);

        SequentialCommandGroup shooterSequence = new SequentialCommandGroup(
                new isAimed(turret).withTimeout(200),
                new transfer(intake, true)
        );

        SequentialCommandGroup safeShooterSequence = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new isAimed(turret),
                        new shooterAtSpeed(shooter)
                ),
                new transfer(intake, true)
        );

        SequentialCommandGroup rampBackCollectSequence = new SequentialCommandGroup(
                new FollowPathCommand(follower, rampCollectPath),
                new WaitCommand(200),
                new FollowPathCommand(follower, rampBackCollectPath),
                new FollowPathCommand(follower, rampFinishCollectPath),
                new ParallelRaceGroup(
                        new WaitCommand(1000),
                        new allBallsDetected(intake)
                ),
                new FollowPathCommand(follower, rampReturnPath)
        );

        SequentialCommandGroup directRampCollectSequence = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new InstantCommand(() -> intake.setCurrentStopEnabled(false)),
                        new FollowPathCommand(follower, directRampCollectPath1)
                ),
                new ParallelRaceGroup(
                        new FollowPathCommand(follower, directRampCollectPath2, true, 0.3),
                        new WaitCommand(1000)
                ),
                new ParallelRaceGroup(
                        new WaitCommand(1000),
                        new safeAllBallsDetected(intake)
                ),
                new InstantCommand(() -> intake.setCurrentStopEnabled(true)),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(800),
                                new InstantCommand(() -> {shooter.setAutoShoot(true);})
                        ),
                        new FollowPathCommand(follower, directRampReturnPath)
                )
        );

        SequentialCommandGroup directRampCollectFarSequence = new SequentialCommandGroup(
                new InstantCommand(() -> intake.setCurrentStopEnabled(false)),
                new FollowPathCommand(follower, directRampCollectPath1),
                new ParallelRaceGroup(
                        new FollowPathCommand(follower, directRampCollectPath2),
                        new WaitCommand(1000)
                ),
                new ParallelRaceGroup(
                        new WaitCommand(1000),
                        new safeAllBallsDetected(intake)
                ),
                new InstantCommand(() -> intake.setCurrentStopEnabled(true)),
                new FollowPathCommand(follower, rampToFarSidePath)
        );

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                new FollowPathCommand(follower, path1),
                shooterSequence,
                new allBallsGone(intake).withTimeout(700),
                new ParallelCommandGroup(
                        new InstantCommand(() -> {shooter.setAutoShoot(false);}),
                        new transfer(intake, false),
                        new FollowPathCommand(follower, path2)
                ),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, path3),
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
                                new InstantCommand(() -> {shooter.setAutoShoot(true);})
                        )
                ),
                shooterSequence,
                new allBallsGone(intake).withTimeout(700),
                new ParallelCommandGroup(
                        new InstantCommand(() -> {shooter.setAutoShoot(false);}),
                        new transfer(intake, false),
                        new FollowPathCommand(follower, path4)
                ),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, path5),
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
                                new InstantCommand(() -> {shooter.setAutoShoot(true);})
                        )
                ),
                shooterSequence,
                new allBallsGone(intake).withTimeout(700),
                new InstantCommand(() -> {shooter.setAutoShoot(false);}),
                new transfer(intake, false),
                directRampCollectSequence,
                shooterSequence,
                new allBallsGone(intake).withTimeout(700),
                new transfer(intake, false),
                directRampCollectSequence,
                shooterSequence,
                new allBallsGone(intake).withTimeout(700),
                new InstantCommand(() -> {shooter.setAutoShoot(false);}),
                new setShooterFromPose(shooter, farSideShootPose, chosenAlliance),
                new transfer(intake, false),
                new FollowPathCommand(follower, lastMarkCollect),
                new FollowPathCommand(follower, lastMarkToShoot),
                safeShooterSequence,
                new allBallsGone(intake).withTimeout(500),
                new setShooterFromPose(shooter, farSideShootPose, chosenAlliance),
                new transfer(intake, false),
                new ParallelCommandGroup(
                        new InstantCommand(() -> intake.setCurrentStopEnabled(false)),
                        new intakeOn1Command(intake),
                        new FollowPathCommand(follower, humanPlayerCollectPath)
                ),
                new ParallelCommandGroup(
                        new InstantCommand(intake::intakeOff),
                        new FollowPathCommand(follower, humanPlayerToShoot)
                ),
                safeShooterSequence
        );

        schedule(autonomousSequence);
    }

    @Override
    public void run() {
        Localization.update();
        super.run();

        loopCounter += 1;

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("At Speed", shooter.atSpeed());
        telemetry.addData("Actual Speed", 0.5*(shooter.getVelA() - shooter.getVelB()));
        telemetry.addData("Loop Times", elapsedtime.milliseconds()/loopCounter);
        telemetry.update();
    }

    @Override
    public void end() {
        RobotConstants.savedPose = follower.getPose();
    }
}

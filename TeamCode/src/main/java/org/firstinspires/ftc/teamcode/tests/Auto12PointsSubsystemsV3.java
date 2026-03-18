package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.globals.Localization.getRedDistance;
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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.PerpetualCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commands.intakeOn1Command;
import org.firstinspires.ftc.teamcode.commands.setShooter;
import org.firstinspires.ftc.teamcode.commands.transfer;
import org.firstinspires.ftc.teamcode.commands.turretAutoAim;
import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Disabled
@Autonomous(name = "Auto12Points")
public class Auto12PointsSubsystemsV3 extends CommandOpMode {

    private Follower follower;
    private Shooter shooter;
    private Intake intake;
    private Turret turret;

    // Headings (radians)
    private static final double H0  = Math.toRadians(0);
    private static final double H45 = Math.toRadians(45);
//    private static final double H90 = Math.toRadians(90);
    TelemetryManager telemetryM;

    // === New points (poses) ===
    private final Pose startPose = new Pose(81.291, 0, Math.toRadians(90));

    // Path1: (start) -> cp -> end
    private final Pose p1CP  = new Pose(81.075, 37.865);
    private final Pose p1End = new Pose(131.874, 35.708, H0);

    // Path2: line end1 -> p2End
    private final Pose p2End = new Pose(85.5228426395939, 27.228, H0);

    // Path3: (p2End) -> cp -> end
    private final Pose p3CP  = new Pose(79.027, 63.663);
    private final Pose p3End = new Pose(133.772, 58.878, H0);

    // Path4: (p3End) -> cp -> end
    private final Pose p4CP  = new Pose(93.241, 58.012);
    private final Pose p4End = new Pose(85.5228426395939, 71.180, H0);

    // Path5: (p4End) -> cp -> end
    private final Pose p5CP  = new Pose(94.193, 84.650);
    private final Pose p5End = new Pose(127.502, 83.553, H0);

    // Path6: line p5End -> p6End (heading 0 -> 45)
    private final Pose p6End = new Pose(85.5228426395939, 83.588, H45);

    // Path7: line p6End -> p7End (tangent heading)
    private final Pose p7End = new Pose(85.5228426395939, 58.001);

    // PathChains
    private PathChain path1, path2, path3, path4, path5, path6, path7;

    private void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(startPose.getX(), startPose.getY()),
                        p1CP,
                        new Pose(p1End.getX(), p1End.getY())
                ))
                .setLinearHeadingInterpolation(Math.toRadians(90), H0)
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(p1End.getX(), p1End.getY()),
                        new Pose(p2End.getX(), p2End.getY())
                ))
                .setConstantHeadingInterpolation(H0)
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(p2End.getX(), p2End.getY()),
                        p3CP,
                        new Pose(p3End.getX(), p3End.getY())
                ))
                .setConstantHeadingInterpolation(H0)
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(p3End.getX(), p3End.getY()),
                        p4CP,
                        new Pose(p4End.getX(), p4End.getY())
                ))
                .setConstantHeadingInterpolation(H0)
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(p4End.getX(), p4End.getY()),
                        p5CP,
                        new Pose(p5End.getX(), p5End.getY())
                ))
                .setConstantHeadingInterpolation(H0)
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(p5End.getX(), p5End.getY()),
                        new Pose(p6End.getX(), p6End.getY())
                ))
                .setLinearHeadingInterpolation(H0, H45)
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(p6End.getX(), p6End.getY()),
                        p7End
                ))
                .setTangentHeadingInterpolation()
                .build();
    }

    @Override
    public void initialize() {
        super.reset();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        intake = new Intake(hardwareMap, telemetryM);
        shooter = new Shooter(hardwareMap, telemetryM);
        turret = new Turret(hardwareMap, telemetryM);
        intake.setStopper(0.6);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        Localization.init(follower, telemetryM);
        super.register(turret);
        super.register(shooter);
        super.register(intake);

        // Keep your command scheduling here (unchanged). Paths are now updated.
        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new turretAutoAim(turret, true),
                        new intakeOn1Command(intake),
                        new setShooter(shooter, (int) speedFromDistance(getRedDistance(startPose)), angleFromDistance(getRedDistance(startPose)))
                ),
//                new setShooter(shooter, (int) speedFromDistance(getRedDistance()), angleFromDistance(getRedDistance())),
                new transfer(intake, true),
                new WaitCommand(3000),
                new transfer(intake,  false),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, path1),
                        new intakeOn1Command(intake)
                ),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, path2),
                        new setShooter(shooter, (int) speedFromDistance(getRedDistance(p2End)), angleFromDistance(getRedDistance(p2End)))
                ),
                new setShooter(shooter, (int) speedFromDistance(getRedDistance()), angleFromDistance(getRedDistance())),
                new transfer(intake, true),
                new WaitCommand(3000),
                new transfer(intake,  false),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, path3),
                        new intakeOn1Command(intake)
                ),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, path4),
                        new setShooter(shooter, (int) speedFromDistance(getRedDistance(p4End)), angleFromDistance(getRedDistance(p4End)))
                ),
                new setShooter(shooter, (int) speedFromDistance(getRedDistance()), angleFromDistance(getRedDistance())),
                new transfer(intake, true),
                new WaitCommand(3000),
                new transfer(intake,  false),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, path5),
                        new intakeOn1Command(intake)
                ),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, path6),
                        new setShooter(shooter, (int) speedFromDistance(getRedDistance(p6End)), angleFromDistance(getRedDistance(p6End)))
                ),
                new setShooter(shooter, (int) speedFromDistance(getRedDistance()), angleFromDistance(getRedDistance())),
                new transfer(intake, true),
                new WaitCommand(3000),
                new ParallelCommandGroup(
                        new transfer(intake,  false),
                        new FollowPathCommand(follower, path7)
                )
        );


        schedule(autonomousSequence);
    }

    @Override
    public void run() {
        super.run();
        Localization.update(); // updates follower + extra calculations for heading velocity

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }
}

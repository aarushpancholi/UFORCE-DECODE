package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.globals.Localization.getGoalDistance;
import static org.firstinspires.ftc.teamcode.globals.Localization.getRedDistance;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.chosenAlliance;
import static org.firstinspires.ftc.teamcode.subsystems.Shooter.angleFromDistance;

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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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

@Autonomous(name = "Blue Far Side Auto 15 Points")
public class BlueFarSide15PointsAuto extends CommandOpMode {

    private Follower follower;
    private Shooter shooter;
    private Intake intake;
    private Turret turret;

    TelemetryManager telemetryM;

    // Headings (radians)
    private static final double H180 = Math.toRadians(180);
    private static final double H135 = Math.toRadians(135);
    private static final double H140 = Math.toRadians(140);

    // --- Poses (variables), like your sample style ---
    private final Pose startPose = new Pose(79.976, 2, Math.toRadians(90)).mirror();

    // Path1: start -> control -> end
    private final Pose p1CP = new Pose(88.5743654822335, 45.71501015228427).mirror();
    private final Pose p1CP2 = new Pose(133.68026142131976, 53.656025380710666).mirror();
    private final Pose p1End = new Pose(133.1899644670051, 68.03887309644671).mirror();

    private final Pose preP1CP  = new Pose(74.075, 24.865).mirror();
    private final Pose preP1End = new Pose(130.874, 35.708, Math.toRadians(0)).mirror();
    private final Pose preP1CPEnd = new Pose(94.96345177664976, 42.98172588832489).mirror();


    // Path2: p1End -> control -> end
    private final Pose p2CP = new Pose(98.558, 62.796).mirror();
    private final Pose p2End = new Pose(85.5228426395939, 75.728, Math.toRadians(45)).mirror();

    // Path3: p2End -> control -> end
    private final Pose p3CP = new Pose(100.746, 85.086).mirror();
    private final Pose p3End = new Pose(130.320, 88.785).mirror();

    // Path4: line p3End -> p4End
    private final Pose p4End = new Pose(85.5228426395939, 75.835).mirror();

    // Path5: line p4End -> p5End
    private final Pose p5End = new Pose(137.336, 58).mirror();

    // Path6: line p5End -> p6End
    private final Pose p6End = new Pose(90.5228426395939, 72.5).mirror();

    // Path11: line p6End -> p11End
    private final Pose p11End = new Pose(87.621, 59.852).mirror();

    // PathChains
    private PathChain setRealStartPath, collectRamp, path1, path2, path3, path4, prePath1, prePath1End, path5, path6, path7, path8, path9, path10, path11;


    private void buildPaths() {
        setRealStartPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, new Pose(
                        p2End.getX(), p2End.getY()
                )))
                .setLinearHeadingInterpolation(Math.toRadians(90), H135)
                .build();
        prePath1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(p2End.getX(), p2End.getY()),
                        preP1CP,
                        new Pose(preP1End.getX(), preP1End.getY())
                ))
                .setLinearHeadingInterpolation(H135, H180)
                .build();
        prePath1End = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(preP1End.getX(), preP1End.getY()),
                        preP1CP,
                        new Pose(p2End.getX(), p2End.getY())
                ))
                .setLinearHeadingInterpolation(H180, H135)
                .build();
        path1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(p2End.getX(), p2End.getY()),
                        p1CP,
                        p1CP2,
                        new Pose(p1End.getX(), p1End.getY())
                ))
                .setLinearHeadingInterpolation(H135, Math.toRadians(160))
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(p1End.getX(), p1End.getY()),
                        p2CP,
                        new Pose(p2End.getX(), p2End.getY())
                ))
                .setLinearHeadingInterpolation(Math.toRadians(160), H135)
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(p2End.getX(), p2End.getY()),
                        p3CP,
                        new Pose(p3End.getX(), p3End.getY())
                ))
                .setLinearHeadingInterpolation(H135, H180)
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(p3End.getX(), p3End.getY()),
                        new Pose(p2End.getX(), p2End.getY())
                ))
                .setLinearHeadingInterpolation(H180, H135)
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(p4End.getX() + 10, p4End.getY()),
                        new Pose(p5End.getX(), p5End.getY())
                ))
                .setLinearHeadingInterpolation(H135, H140)
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(p5End.getX(), p5End.getY()),
                        new Pose(p2End.getX(), p2End.getY())
                ))
                .setConstantHeadingInterpolation(H135)
                .build();

        // Repeated shuttle paths (as in your provided Paths array)
        path7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(p6End.getX(), p6End.getY()),
                        new Pose(p5End.getX(), p5End.getY())
                ))
                .setConstantHeadingInterpolation(H135)
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(p5End.getX(), p5End.getY()),
                        new Pose(p2End.getX(), p2End.getY())
                ))
                .setLinearHeadingInterpolation(Math.toRadians(160), H135)
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(p6End.getX(), p6End.getY()),
                        new Pose(p5End.getX(), p5End.getY())
                ))
                .setConstantHeadingInterpolation(H135)
                .build();

        path10 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(p5End.getX(), p5End.getY()),
                        new Pose(p2End.getX(), p2End.getY())
                ))
                .setConstantHeadingInterpolation(H135)
                .build();

        path11 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(p6End.getX(), p6End.getY()),
                        new Pose(p11End.getX(), p11End.getY())
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
        shooter = new Shooter(hardwareMap, telemetryM);
        turret = new Turret(hardwareMap, telemetryM);
        intake.setStopper(0.45);
        turret.resetTurretEncoder();
//        turret.setAutoAim(true);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();
        Localization.init(follower, telemetryM);

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                new turretAutoAim(turret, true),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, setRealStartPath).setGlobalMaxPower(1),
                        new setShooter(shooter, 1320, angleFromDistance(getGoalDistance(new Pose(p2End.getX(), p2End.getY()), chosenAlliance))),
                        new intakeOn1Command(intake)
                ),
                new isAimed(turret).withTimeout(400),
                new WaitCommand(900),
                new transfer(intake, true)
                        .alongWith(new InstantCommand(() -> intake.intake2Off())),
                new WaitCommand(200),
                new intakeOn2Command(intake),
                new WaitCommand(500),
                new ParallelCommandGroup(
                        new transfer(intake, false)
                                .alongWith(new InstantCommand(() -> intake.intake2Off())),
                        new ParallelRaceGroup(
                                new FollowPathCommand(follower, prePath1),
                                new WaitCommand(3000)
                        )
                ),
                new FollowPathCommand(follower, prePath1End),
                new isAimed(turret),
                new WaitCommand(750),
                new transfer(intake, true)
                        .alongWith(new InstantCommand(() -> intake.intake2Off())),
                new intakeOn2Command(intake),
                new WaitCommand(500),
                new ParallelCommandGroup(
                        new transfer(intake, false)
                                .alongWith(new InstantCommand(() -> intake.intake2Off())),
                        new ParallelRaceGroup(
                                new FollowPathCommand(follower, path1).setGlobalMaxPower(1),
                                new WaitCommand(3000)
                        )
                ),
                new FollowPathCommand(follower, path2),
                new isAimed(turret),
                new WaitCommand(750),
                new transfer(intake, true)
                        .alongWith(new InstantCommand(() -> intake.intake2Off())),
                new intakeOn2Command(intake),
                new WaitCommand(500),
                new ParallelCommandGroup(
                        new transfer(intake, false)
                                .alongWith(new InstantCommand(() -> intake.intake2Off())),
                        new FollowPathCommand(follower, path3).setGlobalMaxPower(1)
                ),
                new FollowPathCommand(follower, path4),
                new isAimed(turret),
                new WaitCommand(750),
                new transfer(intake, true)
                        .alongWith(new InstantCommand(() -> intake.intake2Off())),
                new intakeOn2Command(intake),
                new WaitCommand(500),
                new ParallelCommandGroup(
                        new transfer(intake, false)
                                .alongWith(new InstantCommand(() -> intake.intake2Off())),

                        new FollowPathCommand(follower, path7).setGlobalMaxPower(1)
                ),
                new TurnToCommand(follower, Math.toRadians(20), AngleUnit.RADIANS),
                new ParallelRaceGroup(
                        new allBallsDetected(intake),
                        new WaitCommand(2000)

                ),
                new FollowPathCommand(follower, path8),
                new ParallelCommandGroup(
                        new isAimed(turret)
                ),                new WaitCommand(900),
                new ParallelCommandGroup(
                        new transfer(intake, true),
                        new InstantCommand(() -> intake.intake2On())
                ),

                new WaitCommand(500),
                new FollowPathCommand(follower, path11)

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
        telemetry.addData("distance", getGoalDistance(follower.getPose(), chosenAlliance));
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }
}

package org.firstinspires.ftc.teamcode.main;


import static org.firstinspires.ftc.teamcode.globals.Localization.getGoalDistance;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.chosenAlliance;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.intakeRedRamp;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.redPark;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.redRampCP;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.resetPos;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.savedPose;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;
import static org.firstinspires.ftc.teamcode.subsystems.Shooter.angleFromDistance;
import static org.firstinspires.ftc.teamcode.subsystems.Shooter.speedFromDistance;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.autoIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.intakeOn1Command;
import org.firstinspires.ftc.teamcode.commands.intakeOn2Command;
import org.firstinspires.ftc.teamcode.commands.setShooter;
import org.firstinspires.ftc.teamcode.commands.transfer;
import org.firstinspires.ftc.teamcode.commands.turretAutoAim;
import org.firstinspires.ftc.teamcode.commands.turretStraight;
import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.function.BooleanSupplier;

@Configurable
@TeleOp(name = "Red TeleOp", group = "TeleOp")
public class RedTeleop extends CommandOpMode {
    private Turret turret;
    private double sens = 1.0;
    private Follower follower;
    private Shooter shooter;
    private TelemetryManager telemetry;
    private Intake intake;


    @Override
    public void initialize() {
        super.reset();
        RobotConstants.chosenAlliance = "RED";

        shooter = new Shooter(hardwareMap, telemetry, false);
        turret = new Turret(hardwareMap, telemetry);
        follower = createFollower(hardwareMap);
        follower.setPose(savedPose != null ? savedPose : new Pose(79.976, 0,Math.toRadians(90)));
        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower.startTeleOpDrive(true);
        intake = new Intake(hardwareMap, telemetry);
        Localization.init(follower, telemetry);
        turret.isAutoCode = false;
        turret.resetTurretEncoder();
//        intake.setStopper(0.45);
        shooter.setAutoShoot(true);
        turret.setAutoAim(true);

        super.register(turret);
        super.register(shooter);


        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx toolOp = new GamepadEx(gamepad2);

        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                        .whenPressed(
                                new InstantCommand(() -> turret.resetTurretEncoder())
                        );

        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(
                        new FollowPathCommand(follower, follower.pathBuilder()
                                .addPath(new BezierCurve(
                                        follower.getPose(),
                                        redRampCP,
                                        intakeRedRamp
                                ))
                                .setLinearHeadingInterpolation(follower.getHeading(), intakeRedRamp.getHeading())
                                .build())
                )
                .whenReleased(
                        new InstantCommand(() -> follower.startTeleOpDrive(true) )

                );

        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(
                        new FollowPathCommand(follower, follower.pathBuilder()
                                .addPath(new BezierLine(
                                        follower.getPose(),
                                        redPark
                                ))
                                .setLinearHeadingInterpolation(follower.getHeading(), redPark.getHeading())
                                .build())
                )
                .whenReleased(
                        new InstantCommand(() -> follower.startTeleOpDrive(true) )
                );

        driverOp.getGamepadButton(GamepadKeys.Button.A)
                .whileHeld(
                        new InstantCommand(() -> follower.holdPoint(follower.getPose()))
                ).whenReleased(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> follower.startTeleOpDrive(true) )
                        )
                );

        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new autoIntakeCommand(intake))
                .whenReleased(
                        new InstantCommand(intake::intakeOff).alongWith(new InstantCommand(intake::intakeReset))
                );

        toolOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        new SequentialCommandGroup(
                                new transfer(intake, true),
                                new intakeOn1Command(intake).alongWith(new InstantCommand(() -> intake.intake2On())

                                )
                        ))
                .whenReleased(
                        new SequentialCommandGroup(
                                new transfer(intake, false),
                                new intakeOn1Command(intake).alongWith(new InstantCommand(() -> intake.intake2On())
                                )

                        ));

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new turretStraight(turret));

        toolOp.getGamepadButton(GamepadKeys.Button.TRIANGLE)
                .whenPressed(new intakeOn1Command(intake))
                .whenReleased(
                        new InstantCommand(intake::intakeOff)
                );

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new intakeOn2Command(intake))
                .whenReleased(
                        new InstantCommand(intake::intakeOff)
                );

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(intake::intakeOpposite))
                .whenReleased(
                        new InstantCommand(intake::intakeOff)
                );

        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                        .whileHeld(
                                new ParallelCommandGroup(
                                        new FollowPathCommand(follower,
                                                follower.pathBuilder()
                                                        .addPath(new BezierLine(
                                                                follower.getPose(),
                                                                new Pose(96.934, 98.054))
                                                        )
                                                        .setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(45))
                                                        .build()),
                                        new InstantCommand(() -> {shooter.setAutoShoot(false);}),
                                        new setShooter(shooter, (int) speedFromDistance(getGoalDistance(new Pose(96.934, 98.054), chosenAlliance)), angleFromDistance(getGoalDistance(new Pose(96.934, 98.054), chosenAlliance))),
                                        new InstantCommand(() -> {turret.isAutoCode = true;})

                                        )
                        )
                                .whenReleased(
                                        new ParallelCommandGroup(
                                                new InstantCommand(() -> {shooter.setAutoShoot(true);}),
                                                new InstantCommand(() -> follower.startTeleOpDrive(true)),
                                                new InstantCommand(() -> {turret.isAutoCode = false;}),
                                                new turretAutoAim(turret,true)
                                        )
                                );



        telemetry.update();

    }

    @Override
    public void run() {
        Localization.update();
        super.run();

        if (intake.areAllBallsDetected()) {
            gamepad1.rumble(200);
            gamepad2.rumble(200);
        }

        if (gamepad1.left_trigger > 0.5) {
            follower.setPose(resetPos);
        }


        sens = (gamepad1.right_trigger > 0.3) ? 2.0 : 1.0;
        follower.setTeleOpDrive(-gamepad1.left_stick_y/sens, -gamepad1.left_stick_x/sens, -gamepad1.right_stick_x/sens, true);

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void end() {
        savedPose = follower.getPose();
    }


}




package org.firstinspires.ftc.teamcode.tests;


import static org.firstinspires.ftc.teamcode.globals.RobotConstants.resetPos;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.autoIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.intakeOn1Command;
import org.firstinspires.ftc.teamcode.commands.turretStraight;
import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.vision.AprilTagTracking;

//@Disabled
@Configurable
@Disabled
@TeleOp(name = "TeleopMain Test", group = "TeleOp")
public class TeleopMain extends CommandOpMode {
    private Turret turret;
    private double sens = 1.0;
    private Follower follower;
    private Shooter shooter;
    private TelemetryManager telemetry;
    private Intake intake;
    private AprilTagTracking vision;


    @Override
    public void initialize() {
        super.reset();

        shooter = new Shooter(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        follower = createFollower(hardwareMap);
        follower.setPose(new Pose(135,0,Math.toRadians(90)));
        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower.startTeleOpDrive(true);
        intake = new Intake(hardwareMap, telemetry);
        vision = new AprilTagTracking(hardwareMap);
        Localization.init(follower, telemetry);
        turret.resetTurretEncoder();
        intake.setStopper(0.45);
        shooter.setAutoShoot(true);
        turret.setAutoAim(true);

        super.register(turret);
        super.register(shooter);


        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx toolOp = new GamepadEx(gamepad2);

//        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                        .whileHeld(
//                                new FollowPathCommand(follower, follower.pathBuilder()
//                                        .addPath(new BezierCurve(
//                                                follower.getPose(),
//                                                redRampCP,
//                                                intakeRedRamp
//                                        ))
//                                        .setLinearHeadingInterpolation(follower.getHeading(), intakeRedRamp.getHeading())
//                                        .build())
//                        )
//                .whenReleased(
//                            new InstantCommand(() -> follower.startTeleOpDrive(true) )
//
//                );
//
//        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//                .whileHeld(
//                        new FollowPathCommand(follower, follower.pathBuilder()
//                                .addPath(new BezierLine(
//                                        follower.getPose(),
//                                        redPark
//                                ))
//                                .setLinearHeadingInterpolation(follower.getHeading(), redPark.getHeading())
//                                .build())
//                )
//                        .whenReleased(
//                                new InstantCommand(() -> follower.startTeleOpDrive(true) )
//                        );

        driverOp.getGamepadButton(GamepadKeys.Button.A)
                        .whileHeld(
                                new InstantCommand(() -> follower.holdPoint(follower.getPose()))
                        ).whenReleased(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> follower.startTeleOpDrive(true) )
                        )
                );

        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new autoIntakeCommand(intake))
                .whenReleased(
                        new InstantCommand(intake::intakeOff).alongWith(new InstantCommand(intake::intakeReset))
                );

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                        .whenPressed(new turretStraight(turret));

        toolOp.getGamepadButton(GamepadKeys.Button.TRIANGLE)
                .whenPressed(new intakeOn1Command(intake))
                .whenReleased(
                        new InstantCommand(intake::intakeOff)
                );

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenReleased(
                        new InstantCommand(intake::intakeOff)
                );

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(intake::intakeOpposite))
                .whenReleased(
                        new InstantCommand(intake::intakeOff)
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

    }




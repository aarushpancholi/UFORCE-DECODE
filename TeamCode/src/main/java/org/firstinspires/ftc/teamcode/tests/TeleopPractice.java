package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.globals.Localization.getHeading;
import static org.firstinspires.ftc.teamcode.globals.Localization.getPose;
import static org.firstinspires.ftc.teamcode.globals.Localization.getRedDistance;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.farRedGoalPose;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.intakeBlueRamp;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.intakeRedRamp;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.redGoalPose;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.redPark;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.redRampCP;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.resetPos;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.savedPose;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathPoint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.PIDFController;
import org.firstinspires.ftc.teamcode.vision.AprilTagTracking;

import java.util.List;

@TeleOp(name = "Teleop Practice", group = "TeleOp")
public class TeleopPractice extends OpMode {
    private TelemetryManager telemetryM;
    private PIDFController controller1, controller2;
    private DcMotorEx sh;
    private DcMotorEx sh2;
    public static double targetVelocity, velocity1, velocity2;
    public static double P,I,kV,kS;

    private boolean newShooter = false;
    private Turret turret;
    private Follower follower;
    private Shooter shooter;
    private Intake intake;

    private double yOffset = 0;
    private int speed = 0;
    private boolean autoAim = false;
    private double hoodPos = 0.7;
    private ElapsedTime elapsedtime;
    private AprilTagTracking vision;
    List<LynxModule> allHubs;
    private int loopCounter = 0;

    public void init() {
        elapsedtime = new ElapsedTime();
        elapsedtime.reset();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        sh = hardwareMap.get(DcMotorEx.class, "rsh");
        allHubs = hardwareMap.getAll(LynxModule.class);
        sh2 = hardwareMap.get(DcMotorEx.class, "lsm");
        shooter = new Shooter(hardwareMap, telemetryM);
        sh.setDirection(DcMotorSimple.Direction.FORWARD);
        sh2.setDirection(DcMotorSimple.Direction.FORWARD);
        vision = new AprilTagTracking(hardwareMap);
        turret = new Turret(hardwareMap, telemetryM);
        turret.setAutoAim(true);
//        turret.resetTurretEncoder();
        intake = new Intake(hardwareMap, telemetryM);
        intake.setStopper(0.45);
        follower = createFollower(hardwareMap);
        follower.setStartingPose(savedPose != null ? savedPose : new Pose(135,9,Math.toRadians(90)));
        Localization.init(follower, telemetryM);

        telemetryM.addLine("Initialized");
        telemetryM.update();
    }

    public void start() {
        follower.startTeleOpDrive(true);
        shooter.setHood(hoodPos);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void loop() {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
        double shotDistance = follower.getPose().distanceFrom(redGoalPose);
        double actualShotSpeed = Math.abs(0.5 * (sh.getVelocity() - sh2.getVelocity()));
        double compensatedHoodPos = Shooter.getLowAngleHoodFromDistanceAndSpeed(shotDistance, actualShotSpeed);

        if (gamepad1.xWasPressed()) {
            follower.setPose(vision.getLocalization());
        }

        if (gamepad1.rightBumperWasPressed()) {
            follower.followPath(follower.pathBuilder()
                    .addPath(new BezierLine(
                            follower.getPose(),
                            intakeRedRamp
                    ))
                            .setHeadingInterpolation(
                            HeadingInterpolator.piecewise(
                                    new HeadingInterpolator.PiecewiseNode(
                                            0,
                                            .5,
                                            HeadingInterpolator.tangent
                                    ),
                                    new HeadingInterpolator.PiecewiseNode(
                                            .5,
                                            1.0,
                                            HeadingInterpolator.constant(Math.toRadians(24.5))
                                    )
                            ))
                    .build());
        }
        else if (gamepad1.rightBumperWasReleased()) {
            follower.startTeleOpDrive(true);
        }

        if (gamepad1.leftBumperWasPressed()) {
            follower.followPath(follower.pathBuilder()
                    .addPath(new BezierLine(
                            follower.getPose(),
                            new Pose(87, 74)
                    ))
                    .setHeadingInterpolation(
                            HeadingInterpolator.piecewise(
                                    new HeadingInterpolator.PiecewiseNode(
                                            0,
                                            .8,
                                            HeadingInterpolator.tangent.reverse()
                                    ),
                                    new HeadingInterpolator.PiecewiseNode(
                                            .8,
                                            1.0,
                                            HeadingInterpolator.constant(Math.toRadians(45))
                                    )
                            ))
                    .build());
        }
        else if (gamepad1.leftBumperWasReleased()) {
            follower.startTeleOpDrive(true);
        }



        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
        if (gamepad1.a) {
            hoodPos = compensatedHoodPos;
            shooter.setHood(hoodPos);
            intake.intake1On();
        }
        if (gamepad2.right_bumper) {
            double farExtraInches = Math.max(0, getRedDistance() - 110);
            if(farExtraInches > 0) {
                intake.onSpeed(0.7);
            }
            else {
                intake.intake1On();
            }
        }

        if (gamepad2.rightBumperWasReleased()) {
            intake.intakeOff();
        }

        if (gamepad2.left_bumper) {
            intake.setStopper(0.35);
            double farExtraInches = Math.max(0, getRedDistance() - 110);
            if(farExtraInches > 0) {
                intake.onSpeed(0.7);
            }
            else {
                intake.intake1On();
            }
        }
        else if (gamepad2.leftBumperWasReleased()) {
            intake.intakeOff();
            intake.setStopper(0.45);
        }

//        if (gamepad1.dpadLeftWasReleased()) {
//            hoodPos += 0.025;
//            shooter.setHood(hoodPos);
//        }



            double[] coefficients = Shooter.getCoefficientsFromDistance(shotDistance);
            targetVelocity = coefficients[1];
            if (Math.abs(actualShotSpeed - targetVelocity) > 60) {
                hoodPos = Shooter.getLowAngleHoodFromDistanceAndSpeed(shotDistance, sh.getVelocity());
            } else {
                hoodPos = coefficients[0];
            }
            shooter.setHood(hoodPos);


        velocity1 = sh.getVelocity();
        velocity2 = sh2.getVelocity();
        if (sh.getVelocity() > targetVelocity) {
            sh.setPower(0);
            sh2.setPower(0);
        }
        else if (sh.getVelocity() < targetVelocity) {
            sh.setPower(1);
            sh2.setPower(1);
        }
        else {
            sh.setPower(0);
            sh2.setPower(0);
        }
//        sh.setPower(controller1.calculate(targetVelocity - velocity1, targetVelocity, 0.0));
//        sh2.setPower(controller2.calculate(targetVelocity - velocity2, targetVelocity, 0.0));

        telemetryM.addData("Odometry pose", getPose());
        telemetryM.addData("Limelight pose", vision.getLocalization());
        telemetryM.addData("Loop Times", elapsedtime.milliseconds()/loopCounter);

        loopCounter +=1;

        Localization.update();
        turret.periodic();
        intake.periodic();
        telemetry.update();
        telemetryM.update();
    }
}

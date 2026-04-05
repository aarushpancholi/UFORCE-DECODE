package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.globals.Localization.getHeading;
import static org.firstinspires.ftc.teamcode.globals.Localization.getPose;
import static org.firstinspires.ftc.teamcode.globals.Localization.getRedDistance;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.redGoalPose;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.resetPos;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.PIDFController;
import org.firstinspires.ftc.teamcode.vision.AprilTagTracking;

import java.util.List;

@Configurable
@TeleOp(name = "Shooter tuning v2", group = "TeleOp")
public class ShooterTuningOp2 extends OpMode {
    private TelemetryManager telemetryM;
    private PIDFController controller1, controller2;
    private DcMotorEx sh;
    private DcMotorEx sh2;
    public static double targetVelocity, velocity1, velocity2;
    public static double P,I,kV,kS;
    public double shooterCurrent;

    private boolean newShooter = false;
    private Turret turret;
    private Follower follower;
    private Shooter shooter;
    private Intake intake;

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
        I = 0.3;
        P = 1.5;
        kS = 0.05;
        kV = 0.00039;
        controller1 = new PIDFController(P,I,0.0, 0);
        controller2 = new PIDFController(P,I,0.0, 0);
        turret = new Turret(hardwareMap, telemetryM);
        Shooter.landAngle = Math.toRadians(-7);
        Shooter.shooterDistanceBiasInches = 0;
        redGoalPose  = new Pose(141, 140, Math.toRadians(90));
        turret.resetTurretEncoder();
        intake = new Intake(hardwareMap, telemetryM);
        intake.setStopper(0.45);
        follower = createFollower(hardwareMap);
        follower.setStartingPose(new Pose(135,9,Math.toRadians(90)));
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

        if (gamepad1.optionsWasPressed()) {
            follower.setPose(vision.getLocalization());
        }

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
        if (gamepad1.a) {
//            hoodPos = compensatedHoodPos;
//            shooter.setHood(hoodPos);
            intake.intake1On();
        }
        if (gamepad1.right_bumper) {
            intake.setStopper(0.35);
            intake.engagePTO();
            double farExtraInches = Math.max(0, getRedDistance() - 110);
            if(farExtraInches > 0) {
                intake.onSpeed(0.85);
            }
            else {
                intake.intake1On();
            }
        }

        if (gamepad1.rightBumperWasReleased()) {
            intake.setStopper(0.45);
            intake.intakeOff();
        }
        if (gamepad1.b) {
            intake.intakeOff();
        }
        if (gamepad1.y) {
            turret.resetTurretEncoder();
        }
        shooterCurrent = sh.getCurrent(CurrentUnit.AMPS);
        if (gamepad1.right_trigger > 0.1) {
//            follower.holdPoint(follower.getPose());
            follower.setPose(resetPos);
        }
        if (gamepad1.left_trigger > 0.1) {
            follower.startTeleOpDrive();
        }

        if (gamepad1.left_stick_button) {
            turret.setAutoAim(!autoAim);
        }
        if(gamepad1.dpadUpWasReleased()) {
            targetVelocity += 20;
        }
        if(gamepad1.dpadDownWasReleased()) {
            targetVelocity -= 20;
        }
        if (gamepad1.dpadRightWasReleased()) {
            hoodPos -= 0.025;
            shooter.setHood(hoodPos);
        }
        if (gamepad1.dpadLeftWasReleased()) {
            hoodPos += 0.025;
            shooter.setHood(hoodPos);
        }

        if (gamepad1.rightStickButtonWasReleased()) {
            newShooter = !newShooter;
        }

        if (newShooter) {
            double[] coefficients = Shooter.getCoefficientsFromDistance(shotDistance);
            targetVelocity = coefficients[1] - 40;
            if (Math.abs(actualShotSpeed - targetVelocity) > 30) {
                hoodPos = Shooter.getLowAngleHoodFromDistanceAndSpeed(shotDistance, sh.getVelocity());
            } else {
                hoodPos = coefficients[0];
            }
            shooter.setHood(hoodPos);
        }


        velocity1 = sh.getVelocity();
        velocity2 = sh2.getVelocity();
        if (sh.getVelocity() - 59 > targetVelocity) {
            sh.setPower(-1);
            sh2.setPower(-1);
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

        telemetryM.addData("shooter", targetVelocity);
        telemetryM.addData("hood pos", hoodPos);
        telemetryM.addData("Odometry pose", getPose());
        telemetryM.addData("Limelight pose", vision.getLocalization());
        telemetryM.addData("Loop Times", elapsedtime.milliseconds()/loopCounter);
        telemetry.addData("S3", intake.isBallDetected03());
        telemetry.addData("S2", intake.isBallDetected02());
        telemetry.addData("S1", intake.isBallDetected01());

        loopCounter +=1;

        Localization.update();
        turret.periodic();
        intake.periodic();
        telemetry.update();
        telemetryM.update();
    }
}

package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.globals.Localization.getHeading;
import static org.firstinspires.ftc.teamcode.globals.Localization.getRedDistance;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.redGoalPose;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.PIDFController;

@TeleOp(name = "Shooter tuning v2", group = "TeleOp")
public class ShooterTuningOp2 extends OpMode {
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

    private int speed = 0;
    private boolean autoAim = false;
    private double hoodPos = 0.7;

    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        sh = hardwareMap.get(DcMotorEx.class, "rsh");
        sh2 = hardwareMap.get(DcMotorEx.class, "lsm");
        shooter = new Shooter(hardwareMap, telemetryM, false);
        sh.setDirection(DcMotorSimple.Direction.FORWARD);
        sh2.setDirection(DcMotorSimple.Direction.FORWARD);
        I = 0.3;
        P = 1.5;
        kS = 0.05;
        kV = 0.00039;
        controller1 = new PIDFController(P,I,0.0, 0);
        controller2 = new PIDFController(P,I,0.0, 0);
        turret = new Turret(hardwareMap, telemetryM);
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
        follower.startTeleOpDrive();
        shooter.setHood(hoodPos);
        controller1.setPID(P,I, 0.0);
        controller1.setFeedforward(kV, 0.0, kS);
        controller2.setPID(P,I, 0.0);
        controller2.setFeedforward(kV, 0.0, kS);
    }

    @Override
    public void loop() {
        double shotDistance = follower.getPose().distanceFrom(redGoalPose);
        double actualShotSpeed = Math.abs(0.5 * (sh.getVelocity() - sh2.getVelocity()));
        double compensatedHoodPos = Shooter.getLowAngleHoodFromDistanceAndSpeed(shotDistance, actualShotSpeed);

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
            intake.intake2On();
        }
        if (gamepad1.right_bumper) {
            intake.setStopper(0.3);
            double farExtraInches = Math.max(0, getRedDistance() - 110);
            if(farExtraInches > 0) {
                intake.onSpeed(0.7);
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
            intake.intake2Off();
            intake.intakeOff();
        }
        if (gamepad1.y) {
            turret.resetTurretEncoder();
        }
        if (gamepad1.right_trigger > 0.1) {
            follower.holdPoint(follower.getPose());
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
            targetVelocity = coefficients[1];
            if (Math.abs(actualShotSpeed - targetVelocity) > 40) {
                hoodPos = Shooter.getLowAngleHoodFromDistanceAndSpeed(shotDistance, sh.getVelocity());
            } else {
                hoodPos = coefficients[0];
            }
            shooter.setHood(hoodPos);
        }


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


        telemetry.addData("Target Speed:", targetVelocity);
        telemetry.addData("Actual Shot Speed:", actualShotSpeed);
        telemetry.addData("Hood Pos:", hoodPos);
        telemetry.addData("Hood Angle (deg):", Shooter.getHoodAngleFromPos(hoodPos));
        telemetry.addData("Comp Hood Angle (deg):", Shooter.getHoodAngleFromPos(compensatedHoodPos));
        telemetry.addData("Distance (goal):", shotDistance);
        telemetry.addData("Distance (redDistancePose):", getRedDistance());
        telemetryM.addData("Target Speed:", speed);
        telemetryM.addData("heading", getHeading());
        telemetryM.addData("intake current", intake.getCurrent());
        telemetryM.addData("Robot pos", follower.getPose());
        telemetryM.addData(" Speed:", sh.getVelocity());
        telemetryM.addData("Actual Shot Speed:", actualShotSpeed);
        telemetryM.addData("Hood Pos:", hoodPos);
        telemetryM.addData("Hood Angle:", (hoodPos));
        telemetryM.addData("Comp Hood Angle:", (compensatedHoodPos));
        telemetryM.addData("Distance (goal):", shotDistance);
        telemetryM.addData("Distance (redDistancePose):", getRedDistance());

        Localization.update();
        turret.periodic();
        intake.periodic();
        telemetry.update();
        telemetryM.update();
    }
}

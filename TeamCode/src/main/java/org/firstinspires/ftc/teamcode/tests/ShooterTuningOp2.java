package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.globals.Localization.getRedDistance;
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

    private Turret turret;
    private Follower follower;
    private Shooter shooter;
    private Intake intake;

    private int speed = 0;
    private boolean autoAim = false;
    private double hoodAngle = 0.7;

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
        follower = createFollower(hardwareMap);
        follower.setStartingPose(new Pose(135,9,Math.toRadians(90)));
        Localization.init(follower, telemetryM);

        telemetryM.addLine("Initialized");
        telemetryM.update();
    }

    public void start() {
        follower.startTeleOpDrive();
        shooter.setHood(hoodAngle);
        controller1.setPID(P,I, 0.0);
        controller1.setFeedforward(kV, 0.0, kS);
        controller2.setPID(P,I, 0.0);
        controller2.setFeedforward(kV, 0.0, kS);
    }

    @Override
    public void loop() {
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
        if (gamepad1.a) {
            intake.intake1On();
            intake.intake2On();
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

        if (gamepad1.right_bumper) {
            turret.setAutoAim(!autoAim);
        }
        if(gamepad1.dpadUpWasReleased()) {
            targetVelocity += 20;
        }
        if(gamepad1.dpadDownWasReleased()) {
            targetVelocity -= 20;
        }
        if (gamepad1.dpadRightWasReleased()) {
            hoodAngle -= 0.025;
            shooter.setHood(hoodAngle);
        }
        if (gamepad1.dpadLeftWasReleased()) {
            hoodAngle += 0.025;
            shooter.setHood(hoodAngle);
        }

        if (gamepad1.right_stick_button) {
            targetVelocity = Shooter.getCoefficientsFromDistance(Localization.getRedDistance())[1];
            hoodAngle = Shooter.getCoefficientsFromDistance(Localization.getRedDistance())[0];
            shooter.setHood(hoodAngle);
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
        telemetry.addData("Hood Angle:", hoodAngle);
        telemetry.addData("Distance:", getRedDistance());
        telemetryM.addData("Target Speed:", speed);
        telemetryM.addData(" Speed:", sh.getVelocity());
        telemetryM.addData("Hood Angle:", hoodAngle);
        telemetryM.addData("Distance:", getRedDistance());

        Localization.update();
        turret.periodic();
        telemetry.update();
        telemetryM.update();
    }
}

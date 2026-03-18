package org.firstinspires.ftc.teamcode.tests;


import static org.firstinspires.ftc.teamcode.globals.Localization.getRedDistance;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.maxHoodPos;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.minHoodPos;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;
import static org.firstinspires.ftc.teamcode.subsystems.Shooter.angleFromDistance;
import static org.firstinspires.ftc.teamcode.subsystems.Shooter.speedFromDistance;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.List;

@Configurable
@TeleOp(name = "Shooter test v1", group = "TeleOp")
public class ShooterTeleopTestV1 extends OpMode {
    private Shooter shooter;
    private int speed = 0;
    private Intake intake;
    private Follower follower;
    private Turret turret;
    private TelemetryManager telemetry;
    private List<Double> speeds;

    @Override
    public void init() {
        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        shooter = new Shooter(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        follower = createFollower(hardwareMap);
        follower.setStartingPose(new Pose(79.76,9,Math.toRadians(90)));
        intake = new Intake(hardwareMap, telemetry);
        Localization.init(follower, telemetry);

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        shooter.setTargetEPT(speed);
        follower.startTeleOpDrive();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        telemetry.addData("encoder", turret.getPos());
        if (gamepad1.right_trigger > 0.1) {
            intake.intake2On();
            intake.setStopper(0.3);
        } else {
            intake.intakeOff();
            intake.setStopper(0.45);
            intake.intake1On();
        }

        if (gamepad1.y) {
            intake.intakeOff();
        }

        if (gamepad1.b) {
            turret.resetTurretEncoder();
        }

        if (gamepad1.right_bumper)
        {
            turret.setAutoAim(true);
        }
        if (gamepad1.left_bumper) {
            turret.setAutoAim(false);
        }
        Localization.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
        turret.periodic();


        if (gamepad1.x) {
            intake.intake1On();
        }


        shooter.setTargetEPT(speedFromDistance(getRedDistance()));
        shooter.setHood(angleFromDistance(getRedDistance()));

        shooter.periodic();

        telemetry.update();
    }

    @Override
    public void stop() {
    }
}

package org.firstinspires.ftc.teamcode.tests;


import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;
import static org.firstinspires.ftc.teamcode.subsystems.Turret.targetTicks;
import static org.firstinspires.ftc.teamcode.subsystems.Turret.turretPID;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Configurable
@Disabled
@TeleOp(name = "Shooter PID Tuning", group = "TeleOp")
public class ShooterPIDTuning extends OpMode {
    private TelemetryManager telemetry;
    private Intake intake;
    public static double kP = 0.1;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0002;
    public static int target = 100;


    @Override
    public void init() {
        // Initializes the static Pinpoint wrapper (sets Pinpoint.pinpoint)
//        new Pinpoint(hardwareMap, telemetry);

        // Grab the same localizer so we can call update() explicitly
//        pinpointLocalizer = hardwareMap.get(PinpointLocalizer.class, "pinpoint");

        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        intake = new Intake(hardwareMap, telemetry);

    }

    @Override
    public void start() {
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {

        if (gamepad1.a) {
            intake.intake1On();
        }

        if (gamepad1.leftBumperWasPressed()) {
            intake.engagePTO();
        }

        if (gamepad1.rightBumperWasPressed()) {
            intake.disengagePTO();
        }

        if (gamepad1.b) {
            intake.intakeOff();
        }

        telemetry.update();
    }


    @Override
    public void stop() {
    }
}

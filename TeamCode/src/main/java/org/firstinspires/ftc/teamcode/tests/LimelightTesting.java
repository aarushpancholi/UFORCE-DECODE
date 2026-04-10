package org.firstinspires.ftc.teamcode.tests;


import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.vision.AprilTagTracking;

@Configurable
@Disabled
@TeleOp(name = "ll testing", group = "TeleOp")
public class LimelightTesting extends OpMode {

    public static double X_OFFSET_IN = -17.5;
    public static double Y_OFFSET_IN = 75.0;
    public static double X_SIGN = -1.0;
    public static double Y_SIGN = 1.0;
    public static double HEADING_INPUT_OFFSET_DEG = 0.0;
    public static double POSE_HEADING_OFFSET_DEG = 90.0;
    public static double START_X_IN = 72.0;
    public static double START_Y_IN = 72.0;
    public static double START_HEADING_DEG = 90.0;

    private TelemetryManager telemetry;
    private AprilTagTracking vision;
    private Follower follower;

    private static double comboError(double x, double y, Pose ref) {
        return Math.hypot(x - ref.getX(), y - ref.getY());
    }

    @Override
    public void init() {
        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = createFollower(hardwareMap);
        follower.setStartingPose(new Pose(135,9,Math.toRadians(90)));
        Localization.init(follower, telemetry);
        vision = new AprilTagTracking(hardwareMap);

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleOpDrive(true);
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        Localization.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );

        AprilTagTracking.FIELD_X_OFFSET_IN = X_OFFSET_IN;
        AprilTagTracking.FIELD_Y_OFFSET_IN = Y_OFFSET_IN;
        AprilTagTracking.FIELD_X_SIGN = X_SIGN;
        AprilTagTracking.FIELD_Y_SIGN = Y_SIGN;
        AprilTagTracking.HEADING_OFFSET_DEG = HEADING_INPUT_OFFSET_DEG;
//        AprilTagTracking.POSE_HEADING_OFFSET_DEG = POSE_HEADING_OFFSET_DEG;

        AprilTagTracking.LocalizationDebug debug = vision.getLocalizationDebug();
        Pose odom = Localization.getPose();

        telemetry.addData("LL valid", debug.valid);
        telemetry.addData("Odom pose", String.format("x=%.2f y=%.2f h=%.1fdeg",
                odom.getX(), odom.getY(), Math.toDegrees(odom.getHeading())));
        telemetry.addData("Yaw in/final/raw", String.format("%.1f / %.1f / %.1f deg",
                debug.headingInputDeg, debug.finalYawDeg, debug.rawYawDeg));
        telemetry.addData("Yaw final-input", String.format("%.1f deg",
                AngleUnit.normalizeDegrees(debug.finalYawDeg - debug.headingInputDeg)));

        if (debug.valid) {
            // Check #1: raw MT2 before field-offset mapping
            telemetry.addData("Raw MT2 (in)", String.format("x=%.2f y=%.2f", debug.rawXIn, debug.rawYIn));

            // Check #2: both X mappings
            telemetry.addData("X map (+/-)", String.format("x+=%.2f x-=%.2f", debug.xPlus, debug.xMinus));

            // Check #3: Y mapping and trim
            double currentMappedY = Y_OFFSET_IN + Y_SIGN * debug.rawYIn;
            double yTrimToMatchOdom = odom.getY() - currentMappedY;
            telemetry.addData("Y map (+/-)", String.format("y+=%.2f y-=%.2f", debug.yPlus, debug.yMinus));
            telemetry.addData("Y trim suggestion", String.format("%.2f in", yTrimToMatchOdom));

            double errPP = comboError(debug.xPlus, debug.yPlus, odom);
            double errMP = comboError(debug.xMinus, debug.yPlus, odom);
            double errPM = comboError(debug.xPlus, debug.yMinus, odom);
            double errMM = comboError(debug.xMinus, debug.yMinus, odom);

            telemetry.addData("Err ++ / -+ / +- / --",
                    String.format("%.1f / %.1f / %.1f / %.1f in", errPP, errMP, errPM, errMM));

            String bestCombo = "++";
            double bestErr = errPP;
            if (errMP < bestErr) { bestErr = errMP; bestCombo = "-+"; }
            if (errPM < bestErr) { bestErr = errPM; bestCombo = "+-"; }
            if (errMM < bestErr) { bestErr = errMM; bestCombo = "--"; }
            telemetry.addData("Best sign combo", String.format("%s (%.1f in)", bestCombo, bestErr));

            Pose mapped = debug.transformedPose;
            telemetry.addData("Mapped pose", String.format("x=%.2f y=%.2f h=%.1fdeg",
                    mapped.getX(), mapped.getY(), Math.toDegrees(mapped.getHeading())));
            telemetry.addData("Mapped error", String.format("dx=%.2f dy=%.2f in",
                    mapped.getX() - odom.getX(), mapped.getY() - odom.getY()));
        }

        telemetry.addData("Goal yaw err RED", vision.getYawErrorRadToGoal("RED").orElse(Double.NaN));
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}

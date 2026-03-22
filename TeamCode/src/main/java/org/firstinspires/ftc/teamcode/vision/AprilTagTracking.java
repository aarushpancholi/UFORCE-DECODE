// AprilTagTracking.java  (Limelight-based, crash-proof)
package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.globals.Localization.getHeading;
import static org.firstinspires.ftc.teamcode.globals.Localization.getPose;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.OptionalDouble;

public class AprilTagTracking {

    public static double FIELD_X_OFFSET_IN = -17.5;
    public static double FIELD_Y_OFFSET_IN = 75.0;
    public static double FIELD_X_SIGN = -1.0;
    public static double FIELD_Y_SIGN = 1.0;
    public static double HEADING_OFFSET_DEG = 0.0;

    public IMU imu;

    private final TelemetryManager telemetry;
    private final Limelight3A limelight;

    public AprilTagTracking(HardwareMap hardwareMap) {
        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(60);
        limelight.start();
        limelight.pipelineSwitch(1);

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    public void start() {
    }

    public static class LocalizationDebug {
        public boolean valid;
        public double headingInputDeg;
        public double finalYawDeg;
        public double rawXIn;
        public double rawYIn;
        public double rawYawDeg;
        public double xPlus;
        public double xMinus;
        public double yPlus;
        public double yMinus;
        public Pose transformedPose;
    }

    public OptionalDouble getYawErrorRadToGoal(String goal) {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
            int id = result.getFiducialResults().get(0).getFiducialId();
            if (id == 24 && goal.equals("RED")) {
                telemetry.addData("limelight", result.getTx());
                return OptionalDouble.of(-Math.toRadians(result.getTx()));
            } else if (id == 20 && goal.equals("BLUE")) {
                telemetry.addData("limelight", result.getTx());
                return OptionalDouble.of(-Math.toRadians(result.getTx()));
            } else {
                return OptionalDouble.empty();
            }
        } else {
            return OptionalDouble.empty();
        }
    }

    public LocalizationDebug getLocalizationDebug() {
        LocalizationDebug debug = new LocalizationDebug();

        // Limelight MT2 expects robot yaw in field-aligned degrees.
        debug.headingInputDeg = AngleUnit.normalizeDegrees(Math.toDegrees(getHeading()) + HEADING_OFFSET_DEG);
        limelight.updateRobotOrientation(debug.headingInputDeg);

        LLStatus status = limelight.getStatus();
        debug.finalYawDeg = status != null ? status.getFinalYaw() : 0.0;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose_MT2();

            debug.rawXIn = botpose.getPosition().toUnit(DistanceUnit.INCH).x;
            debug.rawYIn = botpose.getPosition().toUnit(DistanceUnit.INCH).y;
            debug.rawYawDeg = botpose.getOrientation().getYaw(AngleUnit.DEGREES);

            debug.xPlus = 72.0 + debug.rawXIn;
            debug.xMinus = 72.0 - debug.rawXIn;
            debug.yPlus = 72.0 + debug.rawYIn;
            debug.yMinus = 72.0 - debug.rawYIn;

            double xInches = FIELD_X_OFFSET_IN + FIELD_X_SIGN * debug.rawXIn;
            double yInches = FIELD_Y_OFFSET_IN + FIELD_Y_SIGN * debug.rawYIn;
            double headingRad = botpose.getOrientation().getYaw(AngleUnit.RADIANS);
            debug.transformedPose = new Pose(xInches, yInches, headingRad);
            debug.valid = true;
        } else {
            debug.valid = false;
            debug.transformedPose = getPose();
        }

        return debug;
    }

    public Pose getLocalization() {
        double robotYaw = Math.toDegrees(getHeading());
        limelight.updateRobotOrientation(robotYaw);
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose_MT2();

                // Limelight botpose uses meters/degrees, while Pedro Pose uses inches/radians.
                double xInches = botpose.getPosition().toUnit(DistanceUnit.INCH).x + 188.0;
                double yInches = botpose.getPosition().toUnit(DistanceUnit.INCH).y + 75;
                double headingRad = botpose.getOrientation().getYaw(AngleUnit.RADIANS);
                return new Pose(xInches, yInches, headingRad);
            }
        return getPose();
    }
}

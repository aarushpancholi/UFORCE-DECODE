// Turret.java
package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.globals.Localization.getGoalHeadingDiff;
import static org.firstinspires.ftc.teamcode.globals.Localization.getRedDistance;
import static org.firstinspires.ftc.teamcode.globals.Localization.getRedHeadingDiff;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.chosenAlliance;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.blueGoalPose;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.farRedGoalPose;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.maxTurretPos;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.minTurretPos;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.redGoalPose;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.vision.AprilTagTracking;

import java.util.OptionalDouble;

@Configurable
public class Turret extends SubsystemBase {
    public final MotorEx turret;
    private final AprilTagTracking vision;

    public boolean autoAimEnabled = false;
    private boolean positionControlEnabled = false;

    private static final double MIN_TURRET_RAD = Math.toRadians(-180);
    private static final double MAX_TURRET_RAD = Math.toRadians(150);
    private static final double INCHES_TO_METERS = 0.0254;

    public static double maxVelocityLeadDeg = 40.0;

    public static double kP = 0.02;
    public static double kI = 0.0;
    public static double kD = 0.0006;
    public static double kF = 0.0002;
    public boolean isAutoCode = false;
    // PIDF (tune these)
    public static PIDFController turretPID = new PIDFController(
            kP, kI, kD, kF
    );

    private static final int TICKS_TOLERANCE = 2;
    private double maxPower = 1;

    public static int targetTicks = 168;

    public Turret(HardwareMap hardwareMap, TelemetryManager telemetryManager) {
        turret = new MotorEx(hardwareMap, "turret");
        turret.setRunMode(Motor.RunMode.RawPower);
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turret.setInverted(true);

        vision = new AprilTagTracking(hardwareMap);
    }

    public void startVision() {
        vision.start();
    }

    public void straight() {
        setTargetTicks(0);
    }

    public boolean isStraight() {
        return (turret.getCurrentPosition() > -1 && turret.getCurrentPosition() < 1);
    }

    public void setTargetTicks(int ticks) {
        targetTicks = (int) Range.clip(ticks, minTurretPos, maxTurretPos);
        positionControlEnabled = true;
    }

    public void resetTurretEncoder() {
        turret.stopAndResetEncoder();
        targetTicks = 0;
        turretPID.reset();
        positionControlEnabled = false;
    }

    public double getPos() {
        return turret.getCurrentPosition();
    }

    public void setAutoAim(boolean enabled) {
        autoAimEnabled = enabled;
        if (enabled) {
            positionControlEnabled = true;
        } else {
            turret.set(0);
            turretPID.reset();
            positionControlEnabled = false;
        }
    }

    public double getTargetTicksFromPos(Pose pos) {
        double robotHeading = pos.getHeading();
        double robotHeadingPred = normalizeRadians(robotHeading);

        double turretRelHeading = posToHeading(getPos());
        double turretAbsHeading = normalizeRadians(robotHeadingPred + turretRelHeading);

        double localErr;
        double goalBearing;
        if (chosenAlliance == "RED")
        { if (getRedDistance(pos) > 100) {
            goalBearing = farRedGoalPose.minus(pos).getAsVector().getTheta();
        } else {
            goalBearing = redGoalPose.minus(pos).getAsVector().getTheta();
        }}

        else{
            goalBearing = blueGoalPose.minus(pos).getAsVector().getTheta();
        }

        localErr = AngleUnit.normalizeRadians(goalBearing - turretAbsHeading);

        double targetAbsHeading = normalizeRadians(turretAbsHeading + localErr);

        double relToTarget = targetAbsHeading - robotHeadingPred;
        double chosenRel = chooseTurretRelHeading(relToTarget, turretRelHeading);

        targetTicks = (int) Range.clip(headingToPos(chosenRel), minTurretPos, maxTurretPos);

        return targetTicks;
    }

    public void setTurretPos(double ticks) {
        setTargetTicks((int) Math.round(ticks));
    }

    @Override
    public void periodic() {
        if (autoAimEnabled) {
            double robotHeading = Localization.getHeading();
            double robotHeadingPred = normalizeRadians(robotHeading);

            double turretRelHeading = posToHeading(getPos());
            double turretAbsHeading = normalizeRadians(robotHeadingPred + turretRelHeading);

            double localErr = getGoalHeadingDiff(turretAbsHeading, chosenAlliance);

            double velocityLeadRad = getVelocityLeadRad();
            double targetAbsHeading = normalizeRadians(turretAbsHeading + localErr + velocityLeadRad);

            double relToTarget = targetAbsHeading - robotHeadingPred;
            double chosenRel = chooseTurretRelHeading(relToTarget, turretRelHeading);

            setTargetTicks((int) Math.round(headingToPos(chosenRel)));
        }

        if (!autoAimEnabled && !positionControlEnabled) {
            turret.set(0);
            return;
        }

        // PIDF to targetTicks
        turretPID.setSetPoint(targetTicks);

        double current = turret.getCurrentPosition();
        double power = turretPID.calculate(current);
        power = Range.clip(power, -maxPower, maxPower);

//        double err = targetTicks - current;
//        if (Math.abs(err) > TICKS_TOLERANCE && Math.abs(power) < 0.2) {
//            power = Math.copySign(0.2, power);
//        }
//
//
        turret.set(power);
    }

    private double getVelocityLeadRad() {
        double goalDistanceInches = Localization.getGoalDistance(chosenAlliance);
        double[] shooterCoefficients = Shooter.getCoefficientsFromDistance(goalDistanceInches);

        double hoodPos = shooterCoefficients[0];
        double hoodAngleRad = Math.toRadians(Shooter.getHoodAngleFromPos(hoodPos));
        double measuredShotTicksPerSec = Math.abs(0.5 * (Shooter.velocity1 - Shooter.velocity2));
        double shooterTicksPerSec = measuredShotTicksPerSec > 1.0
                ? measuredShotTicksPerSec
                : Math.max(Shooter.targetVelocity, shooterCoefficients[1]);
        double shooterSpeedMps = Shooter.getShooterSpeedFromTicks(shooterTicksPerSec);

        Pose goalPose = blueGoalPose;
        if (chosenAlliance.equals("RED")) {
            goalPose = goalDistanceInches > 100.0 ? farRedGoalPose : redGoalPose;
        }

        Vector robotToGoal = goalPose
                .minus(Localization.getPose())
                .getAsVector();
        Vector robotVelocity = Localization.getVelocity();

        double coordinateTheta = robotVelocity.getTheta() - robotToGoal.getTheta();
        double robotSpeedMps = robotVelocity.getMagnitude() * INCHES_TO_METERS;
        double parallelComponent = -Math.cos(coordinateTheta) * robotSpeedMps;
        double perpendicularComponent = Math.sin(coordinateTheta) * robotSpeedMps;

        double horizontalLaunchSpeedMps = shooterSpeedMps * Math.cos(hoodAngleRad);
        double lateralDemandSq = perpendicularComponent * perpendicularComponent;
        double horizontalSpeedSq = horizontalLaunchSpeedMps * horizontalLaunchSpeedMps;
        if (horizontalSpeedSq <= lateralDemandSq + 1e-6) {
            return 0.0;
        }

        // The compensated shooter solution already bakes chassis parallel velocity into the
        // required horizontal launch magnitude. Recover the along-goal component from that
        // magnitude instead of adding parallel velocity a second time.
        double launcherParallelComponent = Math.sqrt(horizontalSpeedSq - lateralDemandSq);

        // Sign is inverted so turret leads in the physical/camera frame correctly.
        double leadRad = -Math.atan2(perpendicularComponent, launcherParallelComponent);
        double maxLeadRad = Math.toRadians(maxVelocityLeadDeg);
        return Range.clip(leadRad, -maxLeadRad, maxLeadRad);
    }

    public boolean isAimed() {
        return Math.abs(targetTicks - turret.getCurrentPosition()) <= TICKS_TOLERANCE;
    }

    private double posToHeading(double posTicks) {
        double pos = Range.clip(posTicks, minTurretPos, maxTurretPos);
        double t = (pos - minTurretPos) / (maxTurretPos - minTurretPos); // 0..1
        return MIN_TURRET_RAD + t * (MAX_TURRET_RAD - MIN_TURRET_RAD);
    }

    private double headingToPos(double headingRad) {
        double h = Range.clip(headingRad, MIN_TURRET_RAD, MAX_TURRET_RAD);
        double t = (h - MIN_TURRET_RAD) / (MAX_TURRET_RAD - MIN_TURRET_RAD); // 0..1
        return minTurretPos + t * (maxTurretPos - minTurretPos);
    }


    private double chooseTurretRelHeading(double relRad, double currentTurretRelRad) {
        double base = normalizeRadians(relRad);

        double[] candidates = new double[] { base, base + 2.0 * Math.PI, base - 2.0 * Math.PI };

        double bestInRange = Double.NaN;
        double bestDist = Double.POSITIVE_INFINITY;

        for (double c : candidates) {
            if (c >= MIN_TURRET_RAD && c <= MAX_TURRET_RAD) {
                double d = Math.abs(c - currentTurretRelRad);
                if (d < bestDist) {
                    bestDist = d;
                    bestInRange = c;
                }
            }
        }
        if (!Double.isNaN(bestInRange)) return bestInRange;

        double bestClamped = 0.0;
        bestDist = Double.POSITIVE_INFINITY;

        for (double c : candidates) {
            double clamped = Range.clip(c, MIN_TURRET_RAD, MAX_TURRET_RAD);
            double d = Math.abs(clamped - currentTurretRelRad);
            if (d < bestDist) {
                bestDist = d;
                bestClamped = clamped;
            }
        }
        return bestClamped;
    }
}

package org.firstinspires.ftc.teamcode.globals;

import static org.firstinspires.ftc.teamcode.globals.RobotConstants.blueGoalPose;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.farRedGoalPose;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.redDistancePose;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.redGoalPose;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Localization {
    private static Follower follower;
    private static TelemetryManager telemetry;

    private static final ElapsedTime timer = new ElapsedTime();
    private static double lastHeading = 0.0;     // rad
    private static double headingVel = 0.0;      // rad/s (filtered)
    private static final double VEL_ALPHA = 0.25; // 0..1 (higher = less smoothing)

    public static void init(Follower f, TelemetryManager telemetryManager) {
        follower = f;
        telemetry = telemetryManager;

        follower.update();
        lastHeading = follower.getHeading();
        headingVel = 0.0;
        timer.reset();
    }

    public static void update() {
            follower.update();
            headingVel = follower.getAngularVelocity(); // rad/s, no extra lag
            lastHeading = follower.getHeading();
    }

    public static void updateNoFollower() {
        double dt = timer.seconds();
        timer.reset();

        double h = follower.getHeading();
        telemetry.addData("dt", dt);

        if (dt > 1e-3) {
            double dh = AngleUnit.normalizeRadians(h - lastHeading);
            double rawVel = dh / dt;
            headingVel = (1.0 - VEL_ALPHA) * headingVel + VEL_ALPHA * rawVel;
            telemetry.addData("dh", dh);
        }

        lastHeading = h;
    }

    public static double getHeading() {
        return follower.getHeading();
    }

    public static double getHeadingVelocity() {
        return headingVel;
    }

    public static double getX() {
        return follower.getPose().getX();
    }

    public static Vector getVelocity() { return follower.getVelocity();}

    public static Pose getPose() {return follower.getPose();}

    public static double getZLateral() {
        return follower.getPose().getY();
    }

    public static double getRedDistance() {
        return follower.getPose().distanceFrom(redDistancePose);
    }

    public static double getRedDistance(Pose loc) {
        return loc.distanceFrom(redDistancePose);
    }

    public static double getBlueDistance() {
        return follower.getPose().distanceFrom(blueGoalPose);
    }

    public static double getBlueDistance(Pose loc) {
        return loc.distanceFrom(blueGoalPose);
    }

    public static double getBlueHeadingDiff(double turretAbsHeading) {
        Pose robot = follower.getPose();
        double goalBearing = blueGoalPose.minus(robot).getAsVector().getTheta();
        return AngleUnit.normalizeRadians(goalBearing - turretAbsHeading);
    }

    public static double getRedHeadingDiff(double turretAbsHeading) {
        Pose robot = follower.getPose();
        double goalBearing;
        if (getRedDistance() > 100) {
            goalBearing = farRedGoalPose.minus(robot).getAsVector().getTheta();
        } else {
            goalBearing = redGoalPose.minus(robot).getAsVector().getTheta();
        }
        return AngleUnit.normalizeRadians(goalBearing - turretAbsHeading);
    }

    public static double getGoalHeadingDiff(double turretAbsHeading, String goal) {
        if (goal.equals("RED")) {
            return getRedHeadingDiff(turretAbsHeading);
        }
        else if (goal.equals("BLUE")) {
            return getBlueHeadingDiff(turretAbsHeading);
        }
        return getRedHeadingDiff(turretAbsHeading);
    }

    public static double getGoalDistance(Pose loc, String goal) {
        if (goal.equals("RED")) {
            return getRedDistance(loc);
        }
        else if (goal.equals("BLUE")) {
            return getBlueDistance(loc);
        }
        return 0.0;
    }

    public static double getGoalDistance(String goal) {
        if (goal.equals("RED")) {
            return getRedDistance();
        }
        else if (goal.equals("BLUE")) {
            return getBlueDistance();
        }
        return 0.0;
    }
}

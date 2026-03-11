package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Localization.getGoalDistance;
import static org.firstinspires.ftc.teamcode.globals.Localization.getRedDistance;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.chosenAlliance;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.maxEPT;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.maxHoodAngle;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.maxHoodPos;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.minHoodAngle;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.minHoodPos;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.util.PIDFController;

import java.util.Iterator;

public class Shooter extends SubsystemBase {

    private PIDFController controller1, controller2;
    private TelemetryManager telemetry;
    private boolean autoShoot = false;
    private double pos = 0.5;
//    private static double flywheelOffset = 400;
    private DcMotorEx sh;
    private DcMotorEx sh2;
    private ServoEx hood;
    public static double targetVelocity, velocity1, velocity2;
    public static double P,I,kV,kS;
    private static double passThroughPointRadius = 0.127;
    private static boolean isAuto;

    private static InterpLUT distSpeed = new InterpLUT();
    private static InterpLUT distAngle = new InterpLUT();

    public Shooter(HardwareMap hardwareMap, TelemetryManager telemetryManager, boolean isAuto) {
        sh = hardwareMap.get(DcMotorEx.class, "rsh");
        sh2 = hardwareMap.get(DcMotorEx.class, "lsm");
        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        sh.setDirection(DcMotorSimple.Direction.FORWARD);
        hood = new ServoEx(hardwareMap, "hood");
        sh2.setDirection(DcMotorSimple.Direction.FORWARD);
        I = 0.2;
        Shooter.isAuto = isAuto;
        P = 1.3;
        kS = 0.06;
        kV = 0.00039;
        controller1 = new org.firstinspires.ftc.teamcode.util.PIDFController(P,I,0.0, 0);
        controller2 = new org.firstinspires.ftc.teamcode.util.PIDFController(P,I,0.0, 0);


//        distSpeed.add(15.48, 1000);
//        distSpeed.add(35.20, 1125);
//        distSpeed.add(45.55, 1200);
//        distSpeed.add(56.70, 1275);
//        distSpeed.add(72.68, 1350);
//
//        distAngle.add(15.48, 0.65);
//        distAngle.add(35.20, 0.45);
//        distAngle.add(45.55, 0.4);
//        distAngle.add(56.70, 0.4);
//        distAngle.add(72.68, 0.4);
////        distSpeed.add(15.48, 1000);
//        distSpeed.createLUT();
//        distAngle.createLUT();

    }

    @Override
    public void periodic() {
        if (autoShoot) {
            targetVelocity = speedFromDistance(getGoalDistance(chosenAlliance));
            pos = angleFromDistance(getGoalDistance(chosenAlliance));
            setHood(pos);
        }
        telemetry.addData("TargetVel", targetVelocity);
        telemetry.addData("CurrentVel1", velocity1);
        telemetry.addData("CurrentVel2", velocity2);
//        controller1.setPID(P,I, 0.0);
//        controller1.setFeedforward(kV, 0.0, kS);
//        controller2.setPID(P,I, 0.0);
//        controller2.setFeedforward(kV, 0.0, kS);
//        velocity1 = sh.getVelocity();
//        velocity2 = sh2.getVelocity();
        if (sh.getVelocity() >= targetVelocity) {
            sh.setPower(0);
            sh2.setPower(0);
        }
        else if (sh.getVelocity() < targetVelocity) {
            sh.setPower(1);
            sh2.setPower(1);
        }
//        sh.setPower(controller1.calculate(targetVelocity - velocity1, targetVelocity, 0.0));
//        sh2.setPower(controller2.calculate(targetVelocity - velocity2, targetVelocity, 0.0));
        telemetry.update();
    }

    public void setAutoShoot(boolean on) {
        autoShoot = on;
    }

    public void setTargetEPT(double ept) {
        targetVelocity = ept;
    }

    public void setASpeed(double speed) {
        sh.setPower(speed);
    }
    public void setBSpeed(double speed) {
        sh2.setPower(speed);
    }


    public void setHood(double pos) {
        hood.set(pos);
    }

    public double getHoodPos() {
        return hood.get();
    }

    public int getVelA() { return (int) sh.getVelocity(); }
    public int getVelB() { return (int) sh2.getVelocity(); }


    public static double speedFromDistance(double d) {
//        if (d > 0 && d < 47.8) {
//            return (int) 1.41862*d+1041.18911;
//        }
//        else if (d>=47.8 && d < 70) {
//            return (int) 7.70289*d+740.79915;
//        }
//        else if (d>=70) {
//            return (int) (0.00122932*(Math.pow(d, 3)))-(0.318188*(Math.pow(d, 2)))+(30.10425*d+403.64484);
//        }
        if (d>72.68) {
            return distSpeed.get(72.67);
        }
        else if (d<15.48) {
            return distSpeed.get(15.5);
        }
        else { return distSpeed.get(d); }
    }

    public static double angleFromDistance(double d) {
//        if (isAuto) {
//            if (d > 0 && d < 47.8) {
//                return 0.63;
//            }
//            else if (d>=47.8 && d < 70) {
//                return 0.57;
//            }
//            else if (d>=70) {
//                return 0.4;
//            }
//        }
//        else {
//            if (d > 0 && d < 47.8) {
//                return 0.63;
//            }
//            else if (d>=47.8 && d < 70) {
//                return 0.575;
//            }
//            else if (d>=70) {
//                return 0.4;
//            }
//        }
        if (d>72.68) {
            return distAngle.get(72.67);
        }
        else if (d<15.48) {
            return distAngle.get(15.5);
        }
        else { return distAngle.get(d); }
    }

    public static double getHoodPosFromAngle(double angle) {
        double slope = (minHoodPos - maxHoodPos) / (minHoodAngle - maxHoodAngle);
        return slope * (angle - maxHoodAngle) + maxHoodPos;
    }

    public static double getShooterTicksFromSpeed(double speed) {
        return (28*speed/(Math.PI*0.048));
    }

    public static double[] getCoefficientsFromDistance(double d) {
        double g = 9.81;
        double x = (d * 0.0254) - passThroughPointRadius;
        double y = 0.5842;
        double a = Math.toRadians(-30);

        if (x <= 0) {
            return new double[]{angleFromDistance(d), speedFromDistance(d)};
        }

        double minHoodAngleRad = Math.toRadians(Math.min(maxHoodAngle, minHoodAngle));
        double maxHoodAngleRad = Math.toRadians(Math.max(maxHoodAngle, minHoodAngle));
        double hoodAngle = MathUtils.clamp(Math.atan(2 * y / x - Math.tan(a)), minHoodAngleRad, maxHoodAngleRad);

        double denominator = 2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y);
//        if (denominator <= 0) {
//            return new double[]{angleFromDistance(d), speedFromDistance(d)};
//        }

        double ballSpeed = Math.sqrt(g * x * x / denominator);
//        if (Double.isNaN(ballSpeed) || Double.isInfinite(ballSpeed)) {
//            return new double[]{angleFromDistance(d), speedFromDistance(d)};
//        }

        double hoodPos = Range.clip(
                getHoodPosFromAngle(Math.toDegrees(hoodAngle)),
                Math.min(minHoodPos, maxHoodPos),
                Math.max(minHoodPos, maxHoodPos)
        );
        return new double[]{hoodPos, getShooterTicksFromSpeed(ballSpeed)};
    }
}

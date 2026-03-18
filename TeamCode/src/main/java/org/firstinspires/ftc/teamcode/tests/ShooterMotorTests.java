package org.firstinspires.ftc.teamcode.tests;


import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainTest;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Configurable
@TeleOp(name = "shooter Test", group = "TeleOp")
public class ShooterMotorTests extends OpMode {
    private Shooter shooter;
    private Follower follower;
    private TelemetryManager telemetry;
    private DrivetrainTest drivetrain;
    private Intake intake;
    public static double shS1 = 0.0;
    public static double shS2 = 0.0;



    @Override
    public void init() {
        // Initializes the static Pinpoint wrapper (sets Pinpoint.pinpoint)
//        new Pinpoint(hardwareMap, telemetry);

        // Grab the same localizer so we can call update() explicitly
//        pinpointLocalizer = hardwareMap.get(PinpointLocalizer.class, "pinpoint");

        shooter = new Shooter(hardwareMap, telemetry);
        follower = createFollower(hardwareMap);
        drivetrain = new DrivetrainTest(hardwareMap, telemetry);
        follower.setStartingPose(new Pose(140,0,Math.toRadians(90)));
        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        intake = new Intake(hardwareMap, telemetry);
        Localization.init(follower, telemetry);
        shooter.setAutoShoot(false);

        telemetry.addLine("Initialized. Press START to enable auto-aim.");
        telemetry.addLine("Make sure goal pose headings are radians (Math.toRadians(90)).");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleOpDrive();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
//        turret.setAutoAim(true);
//        turret.turretPID.setCoefficients(new PIDFCoefficients(kP, kI, kD, kF));
//        shooter.setASpeed(0.4);
        // Keep localization fresh
        // (If your PinpointLocalizer uses a different update method name, change this line.)
        telemetry.addData("sh1", shooter.getVelA());
        telemetry.addData("sh2", shooter.getVelB());

        shooter.setASpeed(shS1);
        shooter.setBSpeed(shS2);

        if (gamepad1.x) {
            shS1+=0.1;
        }
        if (gamepad1.y) {
            shS1 -= 0.1;
        }
        if (gamepad1.a) {
            shS2 += 0.1;
        }
        if (gamepad1.b) {
            shS2 -= 0.1;
        }

        telemetry.addData("applied power", shS1);
        telemetry.addData("applied power", shS2);
//        shooter.periodic();

        Localization.update();
        drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 1.0);
//        follower.setTeleOpDrive(
//                -gamepad1.left_stick_y,
//                -gamepad1.left_stick_x,
//                -gamepad1.right_stick_x,
//                true
//        );

        // Run your turret auto-aim logic
//        if (gamepad1.a) {
//            turret.resetTurretEncoder();
//        }

//        if (gamepad1.right_bumper) {
//            turret.setAutoAim(true);
////        }
//        intake.testSensors();
//        if (gamepad1.right_trigger > 0.1) {
//            intake.autoIntake();
//        } else {
//            intake.intakeOff();
//            intake.intake2Off();
//        }
//
//        if (intake.areAllBallsDetected()) {
//            gamepad1.rumble(200);
//        }
//
//        if (gamepad1.right_bumper) {
//            intake.intakeReset();
//        }
//
//        turret.periodic();

//         Debug telemetry
//        Pose robotPose = follower.getPose();
//        double robotHeading = Localization.getHeading(); // expected radians

//        double turretRelHeading = turret.encoderToTurretHeading();
//        double turretAbsHeading = AngleUnit.normalizeRadians(robotHeading + turretRelHeading);

        // What your code is using internally
//        double diffFromYourHelper = Localization.getBlueHeadingDiff(turretAbsHeading);
//
//        // Independent sanity check: bearing to goal from (x,y)
//        double dx = RobotConstants.blueGoalPose.getX() - robotPose.getX();
//        double dy = RobotConstants.blueGoalPose.getY() - robotPose.getY();
//        double goalBearingAbs = Math.atan2(dy, dx);
//        double diffFromBearing = AngleUnit.normalizeRadians(goalBearingAbs - turretAbsHeading);
//
//        telemetry.addData("Aimed", turret.isAimed());
//        telemetry.addData("Robot Pose (x,y,hdeg)",
//                String.format("%.1f, %.1f, %.1f",
//                        robotPose.getX(), robotPose.getY(), Math.toDegrees(robotPose.getHeading())));
//        telemetry.addData("Robot Heading (deg)", Math.toDegrees(robotHeading));
//        telemetry.addData("Turret Rel (deg)", Math.toDegrees(turretRelHeading));
//        telemetry.addData("Turret Abs (deg)", Math.toDegrees(turretAbsHeading));
//
//        telemetry.addData("BlueDiff helper (deg)", Math.toDegrees(diffFromYourHelper));
//        telemetry.addData("Goal bearing (deg)", Math.toDegrees(goalBearingAbs));
//        telemetry.addData("Diff bearing (deg)", Math.toDegrees(diffFromBearing));


        telemetry.update();
    }

    @Override
    public void stop() {
//        turret.setAutoAim(false);
    }
}

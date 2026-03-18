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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.List;

@Disabled
@Configurable
@TeleOp(name = "drift stuff", group = "TeleOp")
public class DriftFixing extends OpMode {
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
        follower.setStartingPose(new Pose(135,9,Math.toRadians(90)));
        intake = new Intake(hardwareMap, telemetry);
        Localization.init(follower, telemetry);

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleOpDrive();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
        if (gamepad1.right_bumper) {
            follower.holdPoint(follower.getPose());
        }
        if (gamepad1.left_bumper) {
            follower.startTeleOpDrive();
        }
        if (gamepad1.dpadRightWasReleased()) {
            follower.turnDegrees(10, false);
        }
        if (gamepad1.dpadLeftWasReleased()) {
            follower.turnDegrees(10, true);
        }
        telemetry.addData("heading localizer", Localization.getHeading());
        telemetry.addData("distance localizer", Localization.getRedDistance());
        telemetry.addData("heading follower", follower.getHeading());
        Localization.update();

        telemetry.update();
    }

    @Override
    public void stop() {
    }
}

package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13)
            .forwardZeroPowerAcceleration(-44)
            .lateralZeroPowerAcceleration(-67)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.04, 0, 0.003, 0.02))
//            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.03, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.08, 0.02))
//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.05, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.012, 0, 0.0007, 0.6, 0.02))
//            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.03, 0, 0.00005, 0.6, 0.02))
//            .useSecondaryDrivePIDF(true)
//            .useSecondaryHeadingPIDF(true)
            .centripetalScaling(0.0005);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3.8)
            .strafePodX(-3.8)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fr")
            .leftFrontMotorName("fl")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(80)
            .yVelocity(53.2);
    ;

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}

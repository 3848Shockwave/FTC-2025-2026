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
            .mass(14.515) //kg
            .forwardZeroPowerAcceleration(-45.66)
            .lateralZeroPowerAcceleration(-78.67)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0.00, 0.0, 0.0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(.3,0.0,.01,0.15))
            .headingPIDFCoefficients(new PIDFCoefficients(1.0,0.0,0.0,.01))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(5,0.0,.08,.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(.03,.0003,.004,.6,.001))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.32,0.0,0.000004,.6,.004))
            .centripetalScaling(0.0005)
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("front_right")
            .rightRearMotorName("back_right")
            .leftRearMotorName("back_left")
            .leftFrontMotorName("front_left")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useBrakeModeInTeleOp(true)
            .xVelocity(73.59)
            .yVelocity(54.67)

            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, .7, 1);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-6.88)//inches for now?
            .strafePodX(-1.75)//inches for now?
            .distanceUnit(DistanceUnit.INCH)

            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)

                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
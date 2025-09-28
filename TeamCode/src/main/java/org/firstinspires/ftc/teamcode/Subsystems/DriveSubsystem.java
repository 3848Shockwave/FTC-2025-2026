package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * DriveSubsystem - Controls the robot's mecanum drive system with field-centric control
 */
public class DriveSubsystem extends SubsystemBase {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Follower follower;

    /**
     * Constructor - Initializes all hardware and sets up the drive system
     * @param hardwareMap - FTC's hardware configuration map
     */
    public DriveSubsystem(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, "front left");
        frontRight = hardwareMap.get(DcMotor.class, "front right");
        backLeft = hardwareMap.get(DcMotor.class, "back left");
        backRight = hardwareMap.get(DcMotor.class, "back right");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower = Constants.createFollower(hardwareMap);
    }

    /**
     * Field-centric drive method - the main driving function
     * 
     * @param forward - Forward/backward movement (-1.0 to 1.0, positive = forward)
     * @param strafe  - Left/right movement (-1.0 to 1.0, positive = right)  
     * @param rotate  - Rotational movement (-1.0 to 1.0, positive = clockwise)
     */
    public void driveFieldCentric(double forward, double strafe, double rotate) {
        follower.update();
        double heading = follower.getPose().getHeading();

        // Field-centric transformation
        double rotatedForward = forward * Math.cos(-heading) - strafe * Math.sin(-heading);
        double rotatedStrafe = forward * Math.sin(-heading) + strafe * Math.cos(-heading);

        // Mecanum drive calculations
        double fl = rotatedForward + rotatedStrafe + rotate;
        double fr = rotatedForward - rotatedStrafe - rotate;
        double bl = rotatedForward - rotatedStrafe + rotate;
        double br = rotatedForward + rotatedStrafe - rotate;

        // Power normalization
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)), 
                                           Math.max(Math.abs(bl), Math.abs(br))));

        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    /**
     * Reset the field orientation - makes current robot heading the new "forward" direction
     */
    public void resetHeading() {
        follower.setStartingPose(follower.getPose().copy().setHeading(0));
    }

    /**
     * Get the Pedro Pathing Follower - provides access for autonomous operations
     * @return Follower object for autonomous path following and odometry data
     */
    public Follower getFollower() { 
        return follower; 
    }
}
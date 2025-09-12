package org.firstinspires.ftc.teamcode;

// FTCLib imports
import com.arcrobotics.ftclib.command.SubsystemBase; // Base class for FTCLib subsystems

// Pedro Pathing imports - for field-centric driving and autonomous
import com.pedropathing.follower.Follower;           // Pedro Pathing's main control class
import org.firstinspires.ftc.teamcode.pedroPathing.Constants; // Our Pedro Pathing configuration

// Standard FTC imports
import com.qualcomm.robotcore.hardware.DcMotor;      // Motor control
import com.qualcomm.robotcore.hardware.HardwareMap;  // Hardware configuration access

/**
 * DriveSubsystem - Controls the robot's mecanum drive system with field-centric control
 * 
 * This subsystem encapsulates:
 * - 4 mecanum wheels (frontLeft, frontRight, backLeft, backRight)
 * - Pedro Pathing Follower for odometry and field orientation
 * - Field-centric drive calculations
 * 
 * Field-centric driving means the robot moves relative to the field, not the robot's orientation.
 * For example, pushing the joystick "forward" always moves toward the opponent's side,
 * regardless of which way the robot is facing.
 */
public class DriveSubsystem extends SubsystemBase { // Extends FTCLib's SubsystemBase for command-based programming
    
    // HARDWARE COMPONENTS:
    private DcMotor frontLeft, frontRight, backLeft, backRight; // 4 mecanum drive motors
    private Follower follower; // Pedro Pathing Follower - handles Pinpoint odometry and provides robot position/heading
    
    /**
     * Constructor - Initializes all hardware and sets up the drive system
     * @param hardwareMap - FTC's hardware configuration map
     */
    public DriveSubsystem(HardwareMap hardwareMap) {
        // MOTOR INITIALIZATION - get motors from hardware map using names from Robot Configuration
        frontLeft = hardwareMap.get(DcMotor.class, "front left");   // Must match hardware config name
        frontRight = hardwareMap.get(DcMotor.class, "front right"); // Must match hardware config name
        backLeft = hardwareMap.get(DcMotor.class, "back left");     // Must match hardware config name
        backRight = hardwareMap.get(DcMotor.class, "back right");   // Must match hardware config name
        
        // MOTOR DIRECTION SETUP - Left side motors need to be reversed for mecanum drive
        // This ensures that positive power makes all wheels contribute to forward motion
        frontLeft.setDirection(DcMotor.Direction.REVERSE);  // Reverse left front motor
        backLeft.setDirection(DcMotor.Direction.REVERSE);   // Reverse left back motor
        // Right side motors stay FORWARD (default)
        
        // PEDRO PATHING SETUP - Creates Follower with Pinpoint odometry sensor
        follower = Constants.createFollower(hardwareMap); // Uses our Constants class configuration
        // This sets up the Pinpoint sensor for tracking robot position and heading
    }
    
    /**
     * Field-centric drive method - the main driving function
     * 
     * @param forward - Forward/backward movement (-1.0 to 1.0, positive = forward)
     * @param strafe  - Left/right movement (-1.0 to 1.0, positive = right)  
     * @param rotate  - Rotational movement (-1.0 to 1.0, positive = clockwise)
     */
    public void driveFieldCentric(double forward, double strafe, double rotate) {
        // UPDATE ODOMETRY - Must call this every loop to keep position/heading current
        follower.update(); // Updates Pinpoint sensor readings
        
        // GET CURRENT ROBOT HEADING - from Pinpoint odometry via Pedro Pathing
        double heading = follower.getPose().getHeading(); // Returns heading in radians
        
        // FIELD-CENTRIC TRANSFORMATION - Rotate joystick inputs by robot's heading
        // This makes the robot move relative to the field instead of relative to itself
        double rotatedForward = forward * Math.cos(-heading) - strafe * Math.sin(-heading); // Transform forward component
        double rotatedStrafe = forward * Math.sin(-heading) + strafe * Math.cos(-heading);  // Transform strafe component
        // Note: -heading because we want field-relative, not robot-relative movement
        
        // MECANUM DRIVE CALCULATIONS - Convert movement vectors to individual wheel powers
        // Mecanum drive formula: each wheel contributes to forward, strafe, and rotation
        double fl = rotatedForward + rotatedStrafe + rotate; // Front left wheel power
        double fr = rotatedForward - rotatedStrafe - rotate; // Front right wheel power  
        double bl = rotatedForward - rotatedStrafe + rotate; // Back left wheel power
        double br = rotatedForward + rotatedStrafe - rotate; // Back right wheel power
        
        // POWER NORMALIZATION - Ensure no wheel power exceeds 1.0 (100%)
        // Find the maximum absolute power value
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)), 
                                           Math.max(Math.abs(bl), Math.abs(br))));
        // If any wheel power > 1.0, scale all wheels down proportionally
        // This maintains the movement direction while staying within motor limits
        
        // APPLY MOTOR POWERS - Set the calculated and normalized powers to motors
        frontLeft.setPower(fl / max);  // Apply normalized power to front left
        frontRight.setPower(fr / max); // Apply normalized power to front right
        backLeft.setPower(bl / max);   // Apply normalized power to back left
        backRight.setPower(br / max);  // Apply normalized power to back right
    }
    
    /**
     * Reset the field orientation - makes current robot heading the new "forward" direction
     * This is useful when the robot starts in a different orientation or drifts over time
     */
    public void resetHeading() {
        // Reset the Pinpoint heading to 0 degrees (forward direction)
        follower.setStartingPose(follower.getPose().copy().setHeading(0));
        // copy() creates a new pose object, setHeading(0) makes current direction = forward
    }
    
    /**
     * Get the Pedro Pathing Follower - provides access for autonomous operations
     * @return Follower object for autonomous path following and odometry data
     */
    public Follower getFollower() { 
        return follower; 
        // This allows autonomous code to use the same Follower for path following
        // while teleop uses it for field-centric drive
    }
}
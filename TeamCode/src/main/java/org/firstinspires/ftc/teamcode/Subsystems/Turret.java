package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.OpModes.TeleOpProgram;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.ELCEncoderV2;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.LimelightProcessing;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.RobotConfig;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.TargetInfo;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToState;
import dev.nextftc.hardware.impl.MotorEx;

public class Turret implements Subsystem {

    // Declare for every subsystem
    public static final Turret INSTANCE = new Turret();
    private static double Rkp = 0.003;
    private static double Rkd = 0.0004;
    private static double Rki = .004;
    private static double Rkf = 0.0000275;

    private static double maxPower = 1.0;
    private double xOffset = 0.0;

    boolean testing = false;
    private static double Lvelocity = 600;
    private double testingVelocity = 600;
    boolean manualControl = false;
    double manualAngle = 0.0;
    double manualVelocity = 600.0;
    private static double Lkp = 0.0004;
    private static double Lki = 0.004;
    private static double Lkd = 0.0088;
    private static double Lkf = 0.000452;

    public final LimelightProcessing limelightProcessing = new LimelightProcessing();

    // Hardware
    private final MotorEx launchMotorLeft = new MotorEx("launchMotorLeft").brakeMode();
    private final MotorEx launchMotorRight = new MotorEx("launchMotorRight").brakeMode().reversed();
    private final MotorEx rotateMotor = new MotorEx("rotateMotor").brakeMode();

    private ControlSystem controlSystemTurret = null;
    private ControlSystem controlSystemRotate = null;
    private final List<AprilTagDetection> detectedTags = new ArrayList<>();

    // Constants
    // 0 * 5.625
    double minPosition = 0;
    // -72 degrees * 5.625 ticks/degree = -405 ticks
    double maxPosition = -72 * 5.625;

    private ELCEncoderV2 rotateEncoder;
    private double nextTurretPosition;
    public boolean turretRunning = true;
    private Telemetry telemetry;
    private int launcherTargetID = 0;

    // Goal Coordinates (Inches)
    public final Pose BlueGoal = new Pose(15, 130);
    public final Pose RedGoal = new Pose(127, 130);

    private Turret() {
    }

    public void setXoffset(double x) {
        this.xOffset = x;
    }

    public void setManualControl(boolean manual) {
        this.manualControl = manual;
    }

    public void setManualAnglePower(double angle, double velocity) {
        this.manualAngle = angle;
        this.manualVelocity = velocity;
    }

    public double getRotateMotorPosition() {
        return -rotateEncoder.getTotalDegrees();
    }

    public double getTurretVelocity() {
        return Math.abs((Math.abs(launchMotorLeft.getVelocity()) + Math.abs(launchMotorRight.getVelocity())) / 2);
    }

    public void resetRotateMotorPosition() {
        rotateMotor.setCurrentPosition(0);
    }

    public double getNextTurretPosition() {
        return nextTurretPosition;
    }

    /**
     * Calculates the ABSOLUTE target position for the turret in TICKS.
     * Handles switching between Vision (Limelight) and Blind (Odometry) tracking.
     */
    public double calculatePosition() {
        ArrayList<TargetInfo> targets = limelightProcessing.processTargets();

        // Convert current encoder degrees to Ticks so units match
        double currentTicks = rotateEncoder.getTotalDegrees() * 5.625;

        if (!targets.isEmpty()) {
            // === VISION TRACKING ===
            // Target = Current Position + Vision Offset
            // angle is the tx offset from Limelight
            double angle = limelightProcessing.getTargetInfo().getTargetX();

            // Apply the offset to the current position to get the new absolute target
            // Original logic was (-angle * 5.625) for relative movement
            return currentTicks + (-angle * 5.625);
        } else {
            // === BLIND TRACKING ===
            // Returns the absolute target based on odometry
            return calculateBlindTrackingPosition();
        }
    }

    /**
     * Calculates the target position using Odometry (Pedro Pathing) when vision is lost.
     */
    private double calculateBlindTrackingPosition() {
        // 1. Get Robot Pose (Inches and Degrees)
        Pose robotPose = PedroComponent.follower().getPose();
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeading = robotPose.getHeading(); // Assuming this is in degrees

        // 2. Determine Goal Coordinates based on Alliance
        Pose goalPose = (RobotConfig.alliance == RobotConfig.Alliance.RED) ? RedGoal : BlueGoal;

        // 3. Calculate Vector to Goal
        double dx = goalPose.getX() - robotX;
        double dy = goalPose.getY() - robotY;

        // 4. Calculate Absolute Field Angle to Goal
        // Math.atan2 returns radians, convert to degrees
        double absoluteAngleToGoal = Math.toDegrees(Math.atan2(dy, dx));

        // 5. Calculate Relative Angle (Turret Target)
        // Turret Angle = Goal Angle - Robot Body Heading
        double relativeAngle = normalizeAngle(absoluteAngleToGoal - robotHeading);

        // 6. Convert to Ticks
        // Note: Based on your coordinate system, positive angle = negative ticks
        return -relativeAngle * 5.625;
    }



    /**
     * Calculates distance to goal in CM for blind velocity calculation.
     */
    public double calculateDistanceToGoal() {
        Pose robotPose = PedroComponent.follower().getPose();
        double goalX = (RobotConfig.alliance == RobotConfig.Alliance.RED) ? RedGoal.getX() : BlueGoal.getX();
        double goalY = (RobotConfig.alliance == RobotConfig.Alliance.RED) ? RedGoal.getY() : BlueGoal.getY();

        double dx = goalX - robotPose.getX();
        double dy = goalY - robotPose.getY();
        double distInches = Math.sqrt(dx * dx + dy * dy);

        // Convert to CM
        return distInches * 2.54;
    }

    /**
     * Normalizes an angle to the range [-180, 180] degrees.
     */
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
    /**
     * Clamps the position to the allowed software limits to prevent hardware damage.
     * @param position The target position in ticks
     * @return The clamped position
     */
    private double clampToSafetyBounds(double position) {
        // maxPosition is negative (approx -405), minPosition is 0
        if (position > minPosition) {
            return minPosition;
        } else if (position < maxPosition) {
            return maxPosition;
        }
        return position;
    }
    public double getRealTurretPosition() {
        return -rotateEncoder.getTotalDegrees() / 5.625;
    }

    public void calculateLaunchStrength() {
        if (testing) {
            Lvelocity = testingVelocity;
            controlSystemTurret.setGoal(new KineticState(0.0, Lvelocity, 0.0));
            return;
        }

        limelightProcessing.processTargets();
        double distance = 0;

        if (!limelightProcessing.processTargets().isEmpty()) {
            distance = limelightProcessing.getTargetInfo().getDistance(); // Vision Distance
        } else {
            distance = calculateDistanceToGoal(); // Blind/Odometry Distance
        }

        // Common velocity calculation for both modes
        Lvelocity = (0.0000300785 * Math.pow(distance, 3))
                + (-0.0226083 * Math.pow(distance, 2))
                + (7.13468 * distance)
                + 589.24871;

        if (!limelightProcessing.processTargets().isEmpty()) {
            Vector velocity = PedroComponent.follower().getVelocity();
            // Velocity compensation (optional, currently commented out in original logic)
            // double velocityCompensation = calculateVelocityCompensation(velocity, distance);
            // Lvelocity += velocityCompensation;
        }

        controlSystemTurret.setGoal(new KineticState(0.0, Lvelocity, 0.0));
    }

    private double calculateVelocityCompensation(Vector robotVelocity, double distance) {
        double targetBearing = limelightProcessing.getTargetInfo().getTargetX();
        Pose2D targetDirection = new Pose2D(
                DistanceUnit.MM,
                Math.cos(Math.toRadians(targetBearing)),
                Math.sin(Math.toRadians(targetBearing)), AngleUnit.DEGREES,
                0
        );

        double velocityTowardTarget = (robotVelocity.getXComponent() * targetDirection.getX(DistanceUnit.INCH))
                + (robotVelocity.getYComponent() * targetDirection.getY(DistanceUnit.INCH));

        double compensationFactor = 0.3;
        double distanceScaling = 1.0 / (1.0 + distance / 100.0);

        return velocityTowardTarget * compensationFactor * distanceScaling;
    }

    public void setLaunchMotorSpeed(double power) {
        launchMotorLeft.setPower(power);
    }

    public void initLimelightSystem() {
        if (RobotConfig.alliance == RobotConfig.Alliance.RED) {
            limelightProcessing.initLimelight(4);
            launcherTargetID = 24;
        } else if (RobotConfig.alliance == RobotConfig.Alliance.BLUE) {
            limelightProcessing.initLimelight(3);
            launcherTargetID = 20;
        } else {
            limelightProcessing.initLimelight(3);
        }
    }

    public double calculateYPosition() {
        double y = limelightProcessing.getTargetInfo(21).getTargetY();
        return 366 - 30.5 - y;
    }

    public double getSpeedNeeded() {
        return controlSystemTurret.calculate(new KineticState(0, Lvelocity));
    }

    public Sort.Color[] getColorArray() {
        limelightProcessing.setPipeline(1);
        if (limelightProcessing.getTargetInfo(21) != null) {
            return new Sort.Color[]{Sort.Color.GREEN, Sort.Color.PURPLE, Sort.Color.PURPLE};
        } else if (limelightProcessing.getTargetInfo(22) != null) {
            return new Sort.Color[]{Sort.Color.PURPLE, Sort.Color.GREEN, Sort.Color.PURPLE};
        } else if (limelightProcessing.getTargetInfo(23) != null) {
            return new Sort.Color[]{Sort.Color.PURPLE, Sort.Color.PURPLE, Sort.Color.GREEN};
        } else {
            return null;
        }
    }

    // Deprecated: replaced by calculateBlindTrackingPosition logic inside calculatePosition
    @Deprecated
    public double getBlindTrackingCoordinates() {
        return 0;
    }

    public String getSide() {
        return RobotConfig.alliance.name();
    }

    public ELCEncoderV2 getRotateEncoder() {
        return rotateEncoder;
    }

    @Override
    public void initialize() {
        controlSystemRotate = ControlSystem.builder()
                .posPid(Rkp, Rki, Rkd)
                .basicFF(Rkf)
                .build();
        controlSystemTurret = ControlSystem.builder()
                .velPid(Lkp, Lki, Lkd)
                .basicFF(Lkf)
                .build();
        launchMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rotateEncoder = new ELCEncoderV2(ActiveOpMode.hardwareMap(), "turretEncoder");

        rotateMotor.setPower(0);
        launchMotorLeft.setPower(0);
    }

    @Override
    public void periodic() {
        // Rebuild PID controllers (Consider moving this to initialize() or a tuning method for performance)
        controlSystemRotate = ControlSystem.builder()
                .posPid(Rkp, Rki, Rkd)
                .basicFF(Rkf)
                .build();

        controlSystemTurret = ControlSystem.builder()
                .velPid(Lkp, Lki, Lkd)
                .basicFF(Lkf)
                .build();

        rotateEncoder.updateRotations();

        // Skip logic during Init if not started
        if (!ActiveOpMode.isStarted() && ActiveOpMode.opModeInInit()) {
            rotateMotor.setPower(0);
            launchMotorLeft.setPower(0);
            return;
        }

        // Update Blind Status for Telemetry/State
        boolean targetVisible = !limelightProcessing.processTargets().isEmpty();
        RobotConfig.robotStateTracker.setBlind(!targetVisible);

        // ----------------------------------------
        // CALCULATE TARGET POSITION
        // ----------------------------------------
        if (!manualControl) {
            calculateLaunchStrength(); // Updates Lvelocity based on distance

            // get Absolute Target in Ticks
            nextTurretPosition = calculatePosition();

            // Enforce Safety Boundaries
            nextTurretPosition = clampToSafetyBounds(nextTurretPosition);

        } else {
            // Manual Control Mode
            nextTurretPosition = -manualAngle * 5.625;
            controlSystemTurret.setGoal(new KineticState(0.0, manualVelocity, 0.0));
        }

        // ----------------------------------------
        // EXECUTE CONTROL
        // ----------------------------------------

        // Set goal for rotation (Target Ticks)
        controlSystemRotate.setGoal(new KineticState(nextTurretPosition));

        // CRITICAL FIX: Convert current Degrees to Ticks for PID calculation
        // The PID target is in Ticks (~-400), so the Input must also be in Ticks.
        double currentTicks = rotateEncoder.getTotalDegrees() * 5.625;

        // Safety Check: Only run PID if we are not forcing a reset (Old logic replaced by Clamp)
        // We use the Clamp result directly now.

        double Rpower = controlSystemRotate.calculate(
                new KineticState(currentTicks)
        );

        // Launch Motor Logic
        double Lpower = controlSystemTurret.calculate(
                new KineticState(launchMotorLeft.getCurrentPosition(), getTurretVelocity())
        );

        // Power Clamping
        Rpower = Math.max(-0.8, Math.min(0.8, Rpower));

        // Apply Power
        rotateMotor.setPower(Rpower);
        launchMotorLeft.setPower(-Lpower);
        launchMotorRight.setPower(-Lpower);
    }

    public void rebuildControlSystem(double p, double i, double d, double f, double speed) {
        Rkp = p;
        Rki = i;
        Rkd = d;
        Rkf = f;
        testingVelocity = speed;
    }

    public void setTestSpeed(double speed) {
        testingVelocity = speed;
    }

    public ControlSystem getControlSystemRotate() {
        return controlSystemRotate;
    }
}
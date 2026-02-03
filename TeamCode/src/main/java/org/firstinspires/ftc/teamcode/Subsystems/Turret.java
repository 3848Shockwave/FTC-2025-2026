package org.firstinspires.ftc.teamcode.Subsystems;


import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.ELCEncoderV2;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.LimelightProcessing;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.RobotConfig;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.TargetInfo;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToState;
import dev.nextftc.hardware.impl.MotorEx;


public class Turret implements Subsystem {

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
    private static double Lki =  0.004;
    private static double Lkd =  0.0088;
    private static double Lkf = 0.000452;

    public final LimelightProcessing limelightProcessing = new LimelightProcessing();
    private final MotorEx launchMotorLeft = new MotorEx("launchMotorLeft").brakeMode();
    private final MotorEx launchMotorRight = new MotorEx("launchMotorRight").brakeMode().reversed();
    private final MotorEx rotateMotor = new MotorEx("rotateMotor").brakeMode();

    private ControlSystem controlSystemTurret = null;
    private ControlSystem controlSystemRotate = null;
    private final List<AprilTagDetection> detectedTags = new ArrayList<>();

    // Limits
    double minPosition = 0;
    double maxPosition = -72 * 5.625;

    // ==========================================
    // CRITICAL OFFSETS
    // ==========================================
    // 1. Hardware Offset: Encoder reads -27 when centered.
    private final double HARDWARE_ZERO_OFFSET = -27.0;

    // 2. Mounting Offset: Turret is on the BACK (Opposite to Robot Front)
    // If robot faces 270, Turret faces 90. Difference is 180.
    private final double MOUNTING_OFFSET = 180.0;

    private ELCEncoderV2 rotateEncoder;
    private double nextTurretPosition;
    public  boolean turretRunning = true;
    private Telemetry telemetry;
    private int launcherTargetID = 0;

    // Goals (Based on your Pedro Screenshot: 0,0 is Bottom Left)
    // Blue Goal approx (0, 144) | Red Goal approx (144, 144)
    private final Pose RED_GOAL = new Pose(144, 144);
    private final Pose BLUE_GOAL = new Pose(0, 144);

    // FOLDING constants
    private static final double FOLDING_POSITION = -220;
    private static final double FOLDING_TOLERANCE = 5.0; // degrees
    private static final long FOLDING_TIMEOUT_MS = 2000; // 2 seconds
    private long foldingStartTime = 0;
    private boolean foldingCompleted = false;

    private Turret() {
    }

    public void setXoffset(double x){ this.xOffset = x; }
    public void setManualControl(boolean manual){ this.manualControl = manual; }
    public void setManualAnglePower(double angle, double velocity){ this.manualAngle = angle; this.manualVelocity = velocity; }

    public double getRotateMotorPosition() {
        return -rotateEncoder.getTotalDegrees();
    }

    public double getTurretVelocity(){
        return Math.abs((Math.abs(launchMotorLeft.getVelocity())+Math.abs(launchMotorRight.getVelocity()))/2);
    }

    public void resetRotateMotorPosition() {
        rotateMotor.setCurrentPosition(0);
    }

    public double getNextTurretPosition() {
        return nextTurretPosition;
    }

    public double calculatePosition() {
        // SAFE: Reads the cache we updated in periodic()
        TargetInfo target = limelightProcessing.getTargetInfo();
        if (target != null) {
            double angle = target.getTargetX();
            return (-angle * 5.625);
        }
        return 0;
    }

    public double getRealTurretPosition(){
        return -rotateEncoder.getTotalDegrees()/5.625;
    }

    public void calculateLaunchStrength(boolean hasTarget, double blindBearing) {
        if (testing) {
            Lvelocity = testingVelocity;
            controlSystemTurret.setGoal(new KineticState(0.0, Lvelocity, 0.0));
            return;
        }

        double targetDistance = 0.0;
        double targetBearing = 0.0;

        if (hasTarget) {
            TargetInfo target = limelightProcessing.getTargetInfo();
            if (target != null) {
                targetDistance = target.getDistance();
                targetBearing = target.getTargetX();
            }
        } else {
            Pose robotPose = PedroComponent.follower().getPose();
            Pose goal = (RobotConfig.alliance == RobotConfig.Alliance.RED) ? RED_GOAL : BLUE_GOAL;
            double dx = goal.getX() - robotPose.getX();
            double dy = goal.getY() - robotPose.getY();
            targetDistance = Math.hypot(dx, dy);


            targetBearing = blindBearing;
        }

        if (targetDistance > 0) {
            double rawVelocity = getVelocityFromDistance(targetDistance);
            Vector robotVelocity = PedroComponent.follower().getVelocity();

            double velocityCompensation = calculateVelocityCompensation(robotVelocity, targetDistance, targetBearing);

            Lvelocity = rawVelocity + velocityCompensation;
        } else {
            Lvelocity = 500;
        }
        controlSystemTurret.setGoal(new KineticState(0.0, Lvelocity, 0.0));
    }

    private double calculateVelocityCompensation(Vector robotVelocity, double distance, double targetBearing) {
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
    public void setLaunchMotorSpeed(double power){ launchMotorLeft.setPower(power); }

    public void initLimelightSystem(){
        if(RobotConfig.alliance== RobotConfig.Alliance.RED){ limelightProcessing.initLimelight(4); launcherTargetID=24; }
        else if(RobotConfig.alliance== RobotConfig.Alliance.BLUE){ limelightProcessing.initLimelight(3); launcherTargetID=20; }
        else{ limelightProcessing.initLimelight(3); }
    }

    public double calculateYPosition() {
        double y = limelightProcessing.getTargetInfo(21).getTargetY();
        return 366 - 30.5 - y;
    }

    public double getSpeedNeeded() { return controlSystemTurret.calculate(new KineticState(0, Lvelocity)); }

    public Sort.Color[] getColorArray() {
        limelightProcessing.setPipeline(1);
        if(limelightProcessing.getTargetInfo(21)!=null) return new Sort.Color[]{Sort.Color.GREEN, Sort.Color.PURPLE, Sort.Color.PURPLE};
        else if(limelightProcessing.getTargetInfo(22)!=null) return new Sort.Color[]{Sort.Color.PURPLE, Sort.Color.GREEN, Sort.Color.PURPLE};
        else if (limelightProcessing.getTargetInfo(23) != null) return new Sort.Color[]{Sort.Color.PURPLE, Sort.Color.PURPLE, Sort.Color.GREEN};
        else return null;
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private double getVelocityFromDistance(double distance) {
        return (0.0000300785 * Math.pow(distance, 3))
                + (-0.0226083 * Math.pow(distance, 2))
                + (7.13468 * distance)
                + 589.24871;
    }
    public String getSide(){ return RobotConfig.alliance.name(); }
    public ELCEncoderV2 getRotateEncoder() { return rotateEncoder; }

    // Add a simple turret state machine
    private enum TurretState { INIT, IDLE, VISUAL_TRACKING, BLIND_TRACKING, MANUAL, FOLDING }
    private TurretState turretState = TurretState.INIT;

    @Override
    public void initialize() {
        controlSystemRotate = ControlSystem.builder().posPid(Rkp, Rki, Rkd).basicFF(Rkf).build();
        controlSystemTurret = ControlSystem.builder().velPid(Lkp, Lki, Lkd).basicFF(Lkf).build();
        launchMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rotateEncoder = new ELCEncoderV2(ActiveOpMode.hardwareMap(),"turretEncoder");
        rotateMotor.setPower(0);
        launchMotorLeft.setPower(0);
    }

    @Override
    public void periodic() {
        // Update encoder state every loop
        rotateEncoder.updateRotations();

        // Single limelight read per loop when started
        boolean targetVisible = false;
        if (ActiveOpMode.isStarted()) {
            ArrayList<TargetInfo> targets = limelightProcessing.processTargets();
            targetVisible = !targets.isEmpty();
        }

        // Init safety - stay idle during init
        if (!ActiveOpMode.isStarted() && ActiveOpMode.opModeInInit()) {
            turretState = TurretState.IDLE;
            rotateMotor.setPower(0);
            launchMotorLeft.setPower(0);
            return;
        }

        // Update robot blind state
        RobotConfig.robotStateTracker.setBlind(!targetVisible);

        // --- State transitions ---
        if (manualControl) {
            turretState = TurretState.MANUAL;
        } else if (!ActiveOpMode.isStarted()) {
            turretState = TurretState.IDLE;
        } else if (turretState == TurretState.FOLDING) {
        } else {
            turretState = targetVisible ? TurretState.VISUAL_TRACKING : TurretState.BLIND_TRACKING;
        }

        // --- State actions and shared behavior ---
        double computedNextPosition = rotateEncoder.getTotalDegrees();

        switch (turretState) {
            case MANUAL:
                // Manual override - set explicit angle and velocity
                computedNextPosition = -manualAngle * 5.625;
                controlSystemTurret.setGoal(new KineticState(0.0, manualVelocity, 0.0));
                break;

            case VISUAL_TRACKING:
                // Use camera to track relative angle
                calculateLaunchStrength(true, 0.3);
                computedNextPosition = rotateEncoder.getTotalDegrees() + calculatePosition();
                break;

            case BLIND_TRACKING:
                // Use odometry to aim at goal
                calculateLaunchStrength(false, 0);
                Pose robotPose = PedroComponent.follower().getPose();
                Pose goal = (RobotConfig.alliance == RobotConfig.Alliance.RED) ? RED_GOAL : BLUE_GOAL;
                double dx = goal.getX() - robotPose.getX();
                double dy = goal.getY() - robotPose.getY();
                double absoluteAngleToGoal = Math.toDegrees(Math.atan2(dy, dx));
                double robotHeading = Math.toDegrees(robotPose.getHeading());
                double angleNeeded = normalizeAngle(absoluteAngleToGoal - robotHeading - MOUNTING_OFFSET);
                double targetTicks = (-angleNeeded * 5.625) + (HARDWARE_ZERO_OFFSET * 5.625);
                computedNextPosition = targetTicks;
                break;

            case FOLDING:
                if (foldingStartTime == 0) {
                    foldingStartTime = System.currentTimeMillis();
                    foldingCompleted = false;
                }
                computedNextPosition = FOLDING_POSITION;
                controlSystemRotate.setGoal(new KineticState(FOLDING_POSITION));

                double currentPos = rotateEncoder.getTotalDegrees();
                foldingCompleted = Math.abs(currentPos - FOLDING_POSITION) < FOLDING_TOLERANCE;

                boolean foldingTimeout = (System.currentTimeMillis() - foldingStartTime) > FOLDING_TIMEOUT_MS;
                try {
                    ActiveOpMode.telemetry().addData("TurretState", "FOLDING");
                    ActiveOpMode.telemetry().addData("FoldingCompleted", foldingCompleted);
                    ActiveOpMode.telemetry().addData("FoldingTimeout", foldingTimeout);
                    ActiveOpMode.telemetry().addData("FoldingCurrentPos", currentPos);
                } catch (Exception e) {}
                break;

            case IDLE:
            default:
                // Hold current position
                controlSystemTurret.setGoal(new KineticState(0.0, Lvelocity, 0.0));
                break;
        }

        if (turretState != TurretState.FOLDING) {
            foldingStartTime = 0;
            foldingCompleted = false;
        }

        nextTurretPosition = computedNextPosition;

        // --- Soft limits ---
        if (nextTurretPosition > minPosition) nextTurretPosition = minPosition;
        if (nextTurretPosition < maxPosition) nextTurretPosition = maxPosition;

        if (turretState == TurretState.FOLDING) {
            controlSystemRotate.setGoal(new KineticState(FOLDING_POSITION));
        } else if (nextTurretPosition > -400 && nextTurretPosition <= 0) {
            controlSystemRotate.setGoal(new KineticState(nextTurretPosition));
        } else {
            controlSystemRotate.setGoal(new KineticState(FOLDING_POSITION));
        }

        double Rpower = controlSystemRotate.calculate(
                new KineticState(rotateEncoder.getTotalDegrees())
        );

        double Lpower = controlSystemTurret.calculate(
                new KineticState(launchMotorLeft.getCurrentPosition(), getTurretVelocity())
        );

        try {
            ActiveOpMode.telemetry().addData("TurretState", turretState == null ? "null" : turretState.name());
            ActiveOpMode.telemetry().addData("NextTurretPos", nextTurretPosition);
            ActiveOpMode.telemetry().addData("RotateDegrees", rotateEncoder != null ? rotateEncoder.getTotalDegrees() : Double.NaN);
            ActiveOpMode.telemetry().addData("RotateGoal", controlSystemRotate != null && controlSystemRotate.getGoal() != null ? controlSystemRotate.getGoal().toString() : "null");
            ActiveOpMode.telemetry().addData("LaunchGoalVel", controlSystemTurret != null && controlSystemTurret.getGoal() != null ? String.valueOf(controlSystemTurret.getGoal().component2()) : "null");
        } catch (Exception e) {
            // Defensive: telemetry shouldn't crash the loop; swallow unexpected errors
        }

        // Hard Power Clamps
        if (Rpower > 0.8) Rpower = 0.8;
        if (Rpower < -0.8) Rpower = -0.8;

        rotateMotor.setPower(Rpower);
        launchMotorLeft.setPower(-Lpower);
        launchMotorRight.setPower(-Lpower);
    }

    public void rebuildControlSystem(double p, double i, double d, double f, double speed) {
        Rkp = p; Rki = i; Rkd = d; Rkf = f; testingVelocity = speed;
    }
    public void setTestSpeed(double speed){ testingVelocity = speed; }
    public ControlSystem getControlSystemRotate(){ return controlSystemRotate; }
}

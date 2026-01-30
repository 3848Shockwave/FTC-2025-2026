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
        ArrayList<TargetInfo> targets = limelightProcessing.processTargets();
        double angle = 0;
        if (!targets.isEmpty()) {
            angle = limelightProcessing.getTargetInfo().getTargetX();
            // VISUAL STANDARD:
            // Limelight +Angle (Left) -> Negative Ticks
            return (-angle * 5.625);
        } else {
            return 0;
        }
    }

    public double getRealTurretPosition(){
        return -rotateEncoder.getTotalDegrees()/5.625;
    }

    public void calculateLaunchStrength() {
        if(testing){
            Lvelocity = testingVelocity;
            controlSystemTurret.setGoal(new KineticState(0.0, Lvelocity,0.0));
            return;
        }
        limelightProcessing.processTargets();
        if (!limelightProcessing.processTargets().isEmpty()) {
            double distance = limelightProcessing.getTargetInfo().getDistance();
            Lvelocity =(0.0000300785* Math.pow(distance, 3))
                    +( -0.0226083 * Math.pow(distance, 2))+
                    (7.13468 * distance)
                    + 589.24871;
            Vector velocity = PedroComponent.follower().getVelocity();
            double velocityCompensation = calculateVelocityCompensation(velocity, distance);
            Lvelocity = Lvelocity;
            controlSystemTurret.setGoal(new KineticState(0.0, Lvelocity,0.0));
        }
        else{
            Lvelocity= 600;
            controlSystemTurret.setGoal(new KineticState(0.0, Lvelocity,0.0));
        }
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

    public String getSide(){ return RobotConfig.alliance.name(); }
    public ELCEncoderV2 getRotateEncoder() { return rotateEncoder; }

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
        // Rebuild PID (You can move this to initialize if you want to save performance)
        controlSystemRotate = ControlSystem.builder().posPid(Rkp, Rki, Rkd).basicFF(Rkf).build();
        controlSystemTurret = ControlSystem.builder().velPid(Lkp, Lki, Lkd).basicFF(Lkf).build();

        rotateEncoder.updateRotations();

        if(ActiveOpMode.isStarted()) {
            limelightProcessing.processTargets();
        }

        if (!ActiveOpMode.isStarted()&&ActiveOpMode.opModeInInit()) {
            rotateMotor.setPower(0);
            launchMotorLeft.setPower(0);
            return;
        }

        boolean targetVisible = !limelightProcessing.processTargets().isEmpty();

        if (!targetVisible){
            RobotConfig.robotStateTracker.setBlind(true);
        } else  {
            RobotConfig.robotStateTracker.setBlind(false);
        }

        if(!manualControl) {
            calculateLaunchStrength();

            if(targetVisible){
                // === VISUAL TRACKING (RELATIVE) ===
                // CurrentTicks + Correction.
                // Works because it adjusts based on current error.
                nextTurretPosition = rotateEncoder.getTotalDegrees() + calculatePosition();
            }
            else {
                // === BLIND TRACKING (ABSOLUTE) ===
                Pose robotPose = PedroComponent.follower().getPose();
                Pose goal = (RobotConfig.alliance == RobotConfig.Alliance.RED) ? RED_GOAL : BLUE_GOAL;

                double dx = goal.getX() - robotPose.getX();
                double dy = goal.getY() - robotPose.getY();

                // 1. Absolute Field Angle (0 = East)
                double absoluteAngleToGoal = Math.toDegrees(Math.atan2(dy, dx));

                double robotHeading = Math.toDegrees(robotPose.getHeading());

                // 2. Calculate Angle Needed for Turret
                // Subtract Robot Heading AND Mounting Offset (180 if back-mounted)
                double angleNeeded = normalizeAngle(absoluteAngleToGoal - robotHeading - MOUNTING_OFFSET);

                // 3. Convert to Ticks & Apply Hardware Offset
                // Visual Standard: (-Angle * 5.625)
                // Hardware Offset: Shift Center by -27 degrees
                double targetTicks = (-angleNeeded * 5.625) + (HARDWARE_ZERO_OFFSET * 5.625);

                nextTurretPosition = targetTicks;
            }
        }

        if(manualControl){
            nextTurretPosition = -manualAngle * 5.625;
            controlSystemTurret.setGoal(new KineticState(0.0, manualVelocity,0.0));
        }

        // Safety Clamping
        if (nextTurretPosition > minPosition) nextTurretPosition = minPosition;
        if (nextTurretPosition < maxPosition) nextTurretPosition = maxPosition;

        // Expanded Safety Check: -400 to 0 is your range.
        if (nextTurretPosition > -400 && nextTurretPosition <= 0) {
            controlSystemRotate.setGoal(new KineticState(nextTurretPosition));
        } else {
            controlSystemRotate.setGoal(new KineticState(-220));
        }

        double Rpower = controlSystemRotate.calculate(
                new KineticState(rotateEncoder.getTotalDegrees())
        );

        double Lpower = controlSystemTurret.calculate(new KineticState(launchMotorLeft.getCurrentPosition(), getTurretVelocity()));

        if(Rpower>.8) Rpower=.8;
        if(Rpower<-.8) Rpower = -.8;

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
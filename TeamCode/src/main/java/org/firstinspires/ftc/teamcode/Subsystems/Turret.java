package org.firstinspires.ftc.teamcode.Subsystems;


import com.pedropathing.math.Vector;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

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

    //declare for every subsystem
    public static final Turret INSTANCE = new Turret();
    private static double Rkp = 0.003;
    private static double Rkd = 0.0004;
    private static double Rki = .004;
    private static double Rkf = 0.0000275;

    private static double maxPower =1.0;
    private double xOffset = 0.0;

    boolean testing = false;
    private static double Lvelocity = 600;
    private double testingVelocity = 600;
    boolean manualControl = false;
    double manualAngle =0.0;
    double manualVelocity =600.0;
    private static double Lkp =0.0004;
    private static double Lki =  0.004;
    private static double Lkd =  0.0088;
    private static double Lkf = 0.000452;
    private VoltageSensor voltSensor;



    public final LimelightProcessing limelightProcessing = new LimelightProcessing(); //Creates limelight processing object
    //all the hardware goes here
    private final MotorEx launchMotorLeft = new MotorEx("launchMotorLeft").brakeMode();
    private final MotorEx launchMotorRight = new MotorEx("launchMotorRight").brakeMode().reversed();

    /*
    launcher angle(horizontal): 65  degrees

    Gear Ratio	19.2:1
    Encoder Resolution	537.7 PPR at the Output Shaft

     */
    private final MotorEx rotateMotor = new MotorEx("rotateMotor").brakeMode();
    /*
    13.7 : 1 Ratio, 435 RPM
    Encoder Resolution	384.5 PPR at the Output Shaft

    sku:5203-2402-0014 https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-6mm-d-shaft-435-rpm-36mm-gearbox-3-3-5v-encoder/

    24:135 (gear ratio for the turning plate)
    217.572 mm --> diameter for the plate --> plate Circumference:68.3523 cm

    Total Gear Ratio from motor shaft to plate:
    135/24 = 77.0625 Output shaft encoder, so no need to multiply by 13.7,

    //Encoder ticks per plate rotation:
    135/24* 384.5 = 2162.8125

    __distance moved per encoder count__
    =68.3523/2162.8125 = 0.03160343302
    This represents the degrees that the output 135 tooth pulley moves per encoder tick
     */
    private ControlSystem controlSystemTurret = null;
    private ControlSystem controlSystemRotate =null ;
    private final List<AprilTagDetection> detectedTags = new ArrayList<>();
    double minPosition = 0;//0 * ticksPerDegreeOfRotation; // Starting/default encoder location
    double maxPosition = -72*5.625;//360 * ticksPerDegreeOfRotation; // Convert degrees to encoder counts
    // post-start logic (runs once when start is pressed)
    private ELCEncoderV2 rotateEncoder;
    private double nextTurretPosition;
    public  boolean turretRunning = true;
    private Telemetry telemetry;
    private int launcherTargetID = 0;



    private Turret() {
    }

     public void setXoffset(double x){
        this.xOffset = x;
     }
     public void setManualControl(boolean manual){
        this.manualControl = manual;
     }
     public void setManualAnglePower(double angle, double velocity){
        this.manualAngle = angle;
        this.manualVelocity = velocity;
     }

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
       // ActiveOpMode.telemetry().addData("array return size", targets.size());

        double angle = 0;
        if (!targets.isEmpty()) {
          // ActiveOpMode.telemetry().addData("Target Found,", "Calculating Position");
          //  ActiveOpMode.telemetry().update();
            angle = limelightProcessing.getTargetInfo().getTargetX();//-xOffset;
            return (-angle * 5.625);
        } else {
                // default: difference between current turret reading and (250 * 5.625)
               // double defaultDiff = -(getRealTurretPosition() - (25.0) )* 5.625;
                return 0;
            } //default, may change

    }

    /*
    h0: the height of the launcher
    v0: initial speed of the ball
    theta: the angle of the launcher (relative to the ground)
    d: the distance from the launcher to the target
    h1: the height of the target
    g: the acceleration due to gravity = 9.8 m/s^2

    formula (physical idea situation):
    v_0 = \sqrt{\frac{gd^2}{2 cos^2(\theta)(d\space tan(\theta)+h_0-h)}}

    diameter of fly wheel: 96 mm
    motor shaft diameter is: 8mm -> 4 mm radius

    outLayerSpeed: the speed of the out layer of fly wheel = initial speed of ball
    motorSpeed: the speed of the motor in cm/s
    formula: v_{rim} = v \frac{R}{r_{motor}}

     */
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
            //Lvelocity= 219.6943 * Math.pow(distance,0.361185);//in cm
            Lvelocity =(0.0000300785* Math.pow(distance, 3))
                    +( -0.0226083 * Math.pow(distance, 2))+
                     (7.13468 * distance)
                    + 589.24871;
            //Lvelocity = setTurretVelocity;// for manual control
          Vector velocity = PedroComponent.follower().getVelocity();

            double velocityCompensation = calculateVelocityCompensation(velocity, distance);

            Lvelocity = Lvelocity;// + velocityCompensation;
            controlSystemTurret.setGoal(new KineticState(0.0, Lvelocity,0.0));
        }
        else{
            Lvelocity= 600;
            //Lvelocity = setTurretVelocity; // for manual control
            controlSystemTurret.setGoal(new KineticState(0.0, Lvelocity,0.0));
        }
    }
    private double calculateVelocityCompensation(Vector robotVelocity, double distance) {
        // Get target position/bearing
        double targetBearing = limelightProcessing.getTargetInfo().getTargetX();// adjust to your method

        // Create unit vector pointing toward target
        Pose2D targetDirection = new Pose2D(
                DistanceUnit.MM,
                Math.cos(Math.toRadians(targetBearing)),
                Math.sin(Math.toRadians(targetBearing)), AngleUnit.DEGREES,
                0
        );

        // Dot product to get velocity component along shooting direction
        double velocityTowardTarget = (robotVelocity.getXComponent() * targetDirection.getX(DistanceUnit.INCH))
                + (robotVelocity.getYComponent() * targetDirection.getY(DistanceUnit.INCH));

        // Compensation scales with velocity component
        double compensationFactor = 0.3; // tune empirically
        double distanceScaling = 1.0 / (1.0 + distance / 100.0);

        return velocityTowardTarget * compensationFactor * distanceScaling;
    }

    public void setLaunchMotorSpeed(double power){
        launchMotorLeft.setPower(power);
    }

        /*
        the side of a tile is 61, and the "castle" is 45 degree within a tile
         */
    public void initLimelightSystem(){
        if(RobotConfig.alliance== RobotConfig.Alliance.RED){
            limelightProcessing.initLimelight(4);
            launcherTargetID=24;
        }
        else if(RobotConfig.alliance== RobotConfig.Alliance.BLUE){
            limelightProcessing.initLimelight(3);
            launcherTargetID=20;// Blue Pipeline
        }
        else{
            limelightProcessing.initLimelight(3);
        }
    }


    public double calculateYPosition() {
        double y = limelightProcessing.getTargetInfo(21).getTargetY();
        return 366 - 30.5 - y; //suppose the coordinate system in in cm
    }

    public double getSpeedNeeded() {
        return controlSystemTurret.calculate(
                new KineticState(0, Lvelocity)
        );
    }

    public Sort.Color[] getColorArray() {
        limelightProcessing.setPipeline(1);
        if(limelightProcessing.getTargetInfo(21)!=null){
            return new Sort.Color[]{Sort.Color.GREEN, Sort.Color.PURPLE, Sort.Color.PURPLE};
        }
        else if(limelightProcessing.getTargetInfo(22)!=null){
            return new Sort.Color[]{Sort.Color.PURPLE, Sort.Color.GREEN, Sort.Color.PURPLE};
        }
        else if (limelightProcessing.getTargetInfo(23) != null) {
            return new Sort.Color[]{Sort.Color.PURPLE, Sort.Color.PURPLE, Sort.Color.GREEN};
        }

        else {
            return null;
        }
    }
    public void setPreviousTurretAngle(double angle){

    }

    public double getBlindTrackingCoordinates(){
        double calculatedAngle = 0;
        double robotToGoalAngle =0;
        if(RobotConfig.alliance == RobotConfig.Alliance.RED){

            robotToGoalAngle = Math.toDegrees(Math.atan2((144-PedroComponent.follower().getPose().getY()),( 144-PedroComponent.follower().getPose().getX())));
            if(PedroComponent.follower().getPose().getHeading()>(0)&&PedroComponent.follower().getPose().getHeading()<80){
                double differenceBetweenReal= (robotToGoalAngle)-PedroComponent.follower().getPose().getHeading();
                calculatedAngle =-differenceBetweenReal*5.625;
            }
        }
        else if(RobotConfig.alliance == RobotConfig.Alliance.BLUE){

            robotToGoalAngle =  Math.toDegrees(Math.atan2((144-PedroComponent.follower().getPose().getY()),(0- PedroComponent.follower().getPose().getX())));
            if(PedroComponent.follower().getPose().getHeading()>(100)&&PedroComponent.follower().getPose().getHeading()<170){
               double differenceBetweenReal= (robotToGoalAngle)-PedroComponent.follower().getPose().getHeading();
               calculatedAngle =-differenceBetweenReal*5.625;
            }
        }


        return calculatedAngle;
    }


    public String getSide(){
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
           rotateEncoder = new ELCEncoderV2(ActiveOpMode.hardwareMap(),"turretEncoder");
         voltSensor = ActiveOpMode.hardwareMap().get(VoltageSensor.class, "Control Hub");

       //set default pipeline
        // initialization logic (runs on init)
        rotateMotor.setPower(0);
        // rotateMotor.getRawTicks() this could be interesting?
        //set current position to 0
        launchMotorLeft.setPower(0);
       //initialize limelight processing



    }

    @Override
    public void periodic() {
        controlSystemRotate = ControlSystem.builder()
                .posPid(Rkp, Rki, Rkd)
                .basicFF(Rkf)
               .build();

        controlSystemTurret = ControlSystem.builder()
                .velPid(Lkp, Lki, Lkd)
                .basicFF(Lkf)
                .build();
        rotateEncoder.updateRotations();
        if(ActiveOpMode.isStarted()) {
            limelightProcessing.processTargets();
        }

        // remove after tuning, no need to rebuild control system every loop
        if (!ActiveOpMode.isStarted()&&ActiveOpMode.opModeInInit()) {
           //limelightProcessing.processTargets();
            // add detected tags to telemetry
            rotateMotor.setPower(0);
            launchMotorLeft.setPower(0);
            return;
        }
        if (limelightProcessing.processTargets().isEmpty()){
            RobotConfig.robotStateTracker.setBlind(true);
        } else  {
            RobotConfig.robotStateTracker.setBlind(false);
        }
        if(!manualControl) {
            calculateLaunchStrength();
            if(!limelightProcessing.processTargets().isEmpty() ){
                nextTurretPosition = rotateEncoder.getTotalDegrees() +
                        calculatePosition();
            }
            else if(limelightProcessing.processTargets().isEmpty()){
                nextTurretPosition = -35 * 5.625;
            }
        }
        if(manualControl){
            nextTurretPosition = -manualAngle * 5.625;
            controlSystemTurret.setGoal(new KineticState(0.0, manualVelocity,0.0));
        }

            if (nextTurretPosition > -400 && nextTurretPosition < 0) {
                controlSystemRotate.setGoal(new KineticState(nextTurretPosition));
            } else {
                new RunToState(
                        controlSystemRotate,
                        new KineticState(-220,0.0,0.0),
                        new KineticState(20,0.0,0.0)
                );

                        //controlSystemRotate.setGoal(new KineticState(-220));
            }

        //controlSystemTurret.setGoal(new KineticState(0.0, testingVelocity,0.0));
        // periodic logic (runs every loop)


        //reloads all limelight processing data


        double Rpower = controlSystemRotate.calculate(
                new KineticState(rotateEncoder.getTotalDegrees())
        );


        double Lpower = controlSystemTurret.calculate(new KineticState(launchMotorLeft.getCurrentPosition(), getTurretVelocity()));

      //  ActiveOpMode.telemetry().addData("Voltage",voltSensor.getVoltage());
//        ActiveOpMode.telemetry().addData("Calculated Power",controlSystemTurret.calculate(launchMotorLeft.getState()));
//        ActiveOpMode.telemetry().addData("LaunchMotorState", launchMotorLeft.getState().component2());
//        ActiveOpMode.telemetry().addData("PowerRotate",Rpower);
//        ActiveOpMode.telemetry().addData("Power Turret",Lpower);
//        ActiveOpMode.telemetry().addData("calcLVelocity", Lvelocity);
//        ActiveOpMode.telemetry().addData("calcmovement",calculatePosition());
//        ActiveOpMode.telemetry().addData("calcnext",nextTurretPosition);
//        ActiveOpMode.telemetry().addData("Magic Math Calculation",nextTurretPosition+getBlindTrackingCoordinates());
//
//        if(!limelightProcessing.processTargets().isEmpty()) {
//            ActiveOpMode.telemetry().addData("distance in CM", limelightProcessing.getTargetInfo().getDistance());
//            ActiveOpMode.telemetry().addData("X offset", limelightProcessing.getTargetInfo().getTargetX());
//
//        }
//
//        ActiveOpMode.telemetry().addData("Goal Velocity", controlSystemTurret.getGoal().component2());
//        ///launchMotorLeft.getVelocity();
        //clamp power to limit during testing
        if(Rpower>.8){
            Rpower=.8;
        }
        if(Rpower<-.8){
            Rpower = -.8;
        }
        rotateMotor.setPower(Rpower);
        if(voltSensor.getVoltage()<8){
            Lpower = Lpower * (12.5/voltSensor.getVoltage());
        }
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
    public void setTestSpeed(double speed){
        testingVelocity = speed;
    }
    public ControlSystem getControlSystemRotate(){
        return controlSystemRotate;
    }




}
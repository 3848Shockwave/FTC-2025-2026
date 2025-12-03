package org.firstinspires.ftc.teamcode.Subsystems;


import static org.firstinspires.ftc.teamcode.Subsystems.TurretConstants.distanceperImulseForLunch;
import static org.firstinspires.ftc.teamcode.Subsystems.TurretConstants.flyWheelDiameter;
import static org.firstinspires.ftc.teamcode.Subsystems.TurretConstants.gravityAccalerationValue;
import static org.firstinspires.ftc.teamcode.Subsystems.TurretConstants.motorShaftRadiusForLuncher;
import static org.firstinspires.ftc.teamcode.Subsystems.TurretConstants.ticksPerDegreeOfRotation;
import dev.nextftc.core.commands.Command;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.LimelightProcessing;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;


public class Turret implements Subsystem {

    public enum Side {
        BLUE,
        RED
    }

    private Side side;
    //declare for every subsystem
    public static final Turret INSTANCE = new Turret();
    private static double Rkp = 0.004;
    private static double Rkd = 0.00026;
    private static double Rki = 0.004;
    private static double Rkf = 0.0000275;
    private static double maxPower =1.0;


    private static double Lvelocity = 0.0;
    private static double Lkp =0.007;
    private static double Lki =  .07;
    private static double Lkd =  .004026;
    private static double Lkf = 0.0005;


    public final LimelightProcessing limelightProcessing = new LimelightProcessing(); //Creates limelight processing object
    //all the hardware goes here
    private final MotorEx lunchMotor = new MotorEx("lunchMotor").brakeMode();

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
    double maxPosition = 1000;//360 * ticksPerDegreeOfRotation; // Convert degrees to encoder counts
    // post-start logic (runs once when start is pressed)

    private double nextTurretPosition;
    public  boolean turretRunning = true;
    private Telemetry telemetry;
    private int launcherTargetID = 0;
    private double CurrentturretVelocity;
    public Command  RunTurret = new LambdaCommand().setStart(()->{
        if (side.equals( Side.RED)) {
            limelightProcessing.setPipeline(4);
            launcherTargetID=24;
        }else if (side.equals( Side.BLUE)){
            limelightProcessing.setPipeline(3);
            launcherTargetID=20;
        }
        controlSystemRotate = ControlSystem.builder()
                .posPid(Rkp, Rki, Rkd)
                .basicFF(Rkf)
                .build();
        controlSystemTurret = ControlSystem.builder()
                .velPid(Lkp, Lki, Lkd)
                .basicFF(Lkf)
                .build();
        lunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }).setUpdate(()->{
        controlSystemRotate = ControlSystem.builder()
                .posPid(Rkp, Rki, Rkd)
                .basicFF(Rkf)
                .build();

        controlSystemTurret = ControlSystem.builder()
                .velPid(Lkp, Lki, Lkd)
                .basicFF(Lkf)
                .build();
        limelightProcessing.processTargets();
        calculateLaunchStrength();
        nextTurretPosition = rotateMotor.getCurrentPosition() + calculatePosition();


        ActiveOpMode.telemetry().addData("RunningLoop","");
        // periodic logic (runs every loop)
        if (nextTurretPosition <= maxPosition && nextTurretPosition >= minPosition) {
            controlSystemRotate.setGoal(new KineticState(nextTurretPosition, 50));
        } else if (rotateMotor.getCurrentPosition() < minPosition) {
            controlSystemRotate.setGoal(new KineticState(minPosition, 50));

        } else if (rotateMotor.getCurrentPosition() > maxPosition) {
            controlSystemRotate.setGoal(new KineticState(maxPosition, 50));

        } else {
            controlSystemRotate.setGoal(new KineticState(rotateMotor.getCurrentPosition(), 50));
        }

        double power = controlSystemRotate.calculate(
                new KineticState(rotateMotor.getCurrentPosition())
        );
        double Lpower = controlSystemTurret.calculate(new KineticState(lunchMotor.getCurrentPosition(),lunchMotor.getVelocity()));
        //clamp power to limit during testing
        rotateMotor.setPower(power);
        lunchMotor.setPower(Lpower);
    }).setInterruptible(true).setStop(interrupted->
    {
        rotateMotor.setPower(0);
        lunchMotor.setPower(0);
        turretRunning=false;
    }).requires(this,turretRunning).setName("RunTurret");
    public InstantCommand StopTurret = new InstantCommand(() -> {
        turretRunning=false;
    });


    private Turret() {
    }

    public double getRotateMotorPosition() {
        return rotateMotor.getCurrentPosition();
    }

    public double getTurretVelocity(){
        return lunchMotor.getVelocity();
    }

    public void resetRotateMotorPosition() {
        rotateMotor.setCurrentPosition(0);

    }

    public double getNextTurretPosition() {
        return nextTurretPosition;
    }

    public double calculatePosition() {
        double angle = 0;
        if (!limelightProcessing.processTargets().isEmpty()) {
            ActiveOpMode.telemetry().addData("Target Found,", "Calculating Position");
            angle = limelightProcessing.getTargetInfo().getTargetX()-5;
            //Simple calculation, using the distance from center given by limelight getTargetX, we then divide that by distance per encoder counts
            // This gives us the amount of encoder counts needed to reach desired location.
            // We negate the value because positive angle means target is to the right, so we need to rotate left (negative)
            // return -angle / distancePerImpulseForRotation;
            return -angle * ticksPerDegreeOfRotation;
        } else {
            return 0; //default, may change
        }
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
    public void calculateLaunchStrength() {

        limelightProcessing.processTargets();
        if (!limelightProcessing.processTargets().isEmpty()) {
            double distance = limelightProcessing.getTargetInfo().getDistance();
            Lvelocity= 219.6943 * Math.pow(distance,0.361185);//in cm
            controlSystemTurret.setGoal(new KineticState(0.0, Lvelocity,0.0));
        }
        else{
            Lvelocity= 1345;
            controlSystemTurret.setGoal(new KineticState(0.0, Lvelocity,0.0));
        }
    }
    public void setLaunchMotorSpeed(double power){
        lunchMotor.setPower(power);
    }

        /*
        the side of a tile is 61, and the "castle" is 45 degree within a tile
         */



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
        if(limelightProcessing.getTargetInfo(21) == null){
            limelightProcessing.setPipeline(1);
            if(limelightProcessing.getTargetInfo(22) ==null) {
                limelightProcessing.setPipeline(2);
                if (limelightProcessing.getTargetInfo(23) != null) {
                    return new Sort.Color[]{Sort.Color.PURPLE, Sort.Color.PURPLE, Sort.Color.GREEN};
                }
            }else{
                return new Sort.Color[]{Sort.Color.PURPLE, Sort.Color.GREEN, Sort.Color.PURPLE};
            }
        }else{
            return new Sort.Color[]{Sort.Color.GREEN, Sort.Color.PURPLE, Sort.Color.PURPLE};
        }
        return null;
    }

    public void setSide(String input){
        if (input.equals("red")) {
            side = Side.RED;
            limelightProcessing.setPipeline(4);
            launcherTargetID=24;
        }else if (input.equals("blue")){
            side = Side.BLUE;
            limelightProcessing.setPipeline(3);
            launcherTargetID=20;
        }
    }
    public String getSide(){
        if (side == Side.RED) {
            return "red";
        }else{
            return "blue";
        }
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
        lunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        int defaultPipeline = 3; //set default pipeline
        // initialization logic (runs on init)
        rotateMotor.setPower(0);
        // rotateMotor.getRawTicks() this could be interesting?
        //set current position to 0
        lunchMotor.setPower(0);
        if(side==null){
            side = Side.BLUE;
        }
        limelightProcessing.initLimelight(defaultPipeline); //initialize limelight processing



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

        // remove after tuning, no need to rebuild control system every loop
        if (!ActiveOpMode.isStarted()) {
            limelightProcessing.processTargets();
            // add detected tags to telemetry
            rotateMotor.setPower(0);
            lunchMotor.setPower(0);
            return;
        }

        limelightProcessing.processTargets();
        calculateLaunchStrength();
        nextTurretPosition = rotateMotor.getCurrentPosition() + calculatePosition();


        ActiveOpMode.telemetry().addData("RunningLoop","");
        // periodic logic (runs every loop)
        if (nextTurretPosition <= maxPosition && nextTurretPosition >= minPosition) {
            controlSystemRotate.setGoal(new KineticState(nextTurretPosition, 50));
        } else if (rotateMotor.getCurrentPosition() < minPosition) {
            controlSystemRotate.setGoal(new KineticState(minPosition, 50));

        } else if (rotateMotor.getCurrentPosition() > maxPosition) {
            controlSystemRotate.setGoal(new KineticState(maxPosition, 50));

        } else {
            controlSystemRotate.setGoal(new KineticState(rotateMotor.getCurrentPosition(), 50));
        }


        //reloads all limelight processing data


        double power = controlSystemRotate.calculate(
                new KineticState(rotateMotor.getCurrentPosition())
        );

        double Lpower = controlSystemTurret.calculate(new KineticState(lunchMotor.getCurrentPosition(),lunchMotor.getVelocity()));
        ActiveOpMode.telemetry().addData("Calculated Power",controlSystemTurret.calculate(lunchMotor.getState()));
        ActiveOpMode.telemetry().addData("LaunchMotorState", lunchMotor.getState().component2());
        ActiveOpMode.telemetry().addData("PowerRotate",power);
        ActiveOpMode.telemetry().addData("Power Turret",Lpower);
        ActiveOpMode.telemetry().addData("calcLVelocity", Lvelocity);
        if(!limelightProcessing.processTargets().isEmpty()) {
            ActiveOpMode.telemetry().addData("distance in CM", limelightProcessing.getTargetInfo().getDistance());
        }
        ActiveOpMode.telemetry().addData("Goal Velocity", controlSystemTurret.getGoal().component2());
        lunchMotor.getVelocity();
        //clamp power to limit during testing
        rotateMotor.setPower(power);
        lunchMotor.setPower(Lpower);



    }
    public void rebuildControlSystem(double p, double i, double d, double f, double power) {
        Rkp = p;
        Rki = i;
        Rkd = d;
        Rkf = f;
        maxPower = power;
    }



}
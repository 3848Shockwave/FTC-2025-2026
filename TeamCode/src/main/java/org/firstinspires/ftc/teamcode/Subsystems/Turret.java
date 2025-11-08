    package org.firstinspires.ftc.teamcode.Subsystems;


    import static org.firstinspires.ftc.teamcode.Subsystems.TurretConstants.distancePerImpulseForRotation;
    import static org.firstinspires.ftc.teamcode.Subsystems.TurretConstants.distanceperImulseForLunch;
    import static org.firstinspires.ftc.teamcode.Subsystems.TurretConstants.flyWheelDiameter;
    import static org.firstinspires.ftc.teamcode.Subsystems.TurretConstants.gravityAccalerationValue;
    import static org.firstinspires.ftc.teamcode.Subsystems.TurretConstants.motorShaftRadiusForLuncher;

    import dev.nextftc.control.ControlSystem;
    import dev.nextftc.ftc.ActiveOpMode;

    import org.firstinspires.ftc.robotcore.external.Telemetry;

    import org.firstinspires.ftc.teamcode.Subsystems.Helpers.LimelightProcessing;
    import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


    import java.util.ArrayList;
    import java.util.List;

    import dev.nextftc.control.KineticState;
    import dev.nextftc.core.subsystems.Subsystem;
    import dev.nextftc.hardware.impl.MotorEx;


    public class Turret implements Subsystem {
    //declare for every subsystem
    public static final Turret INSTANCE = new Turret();
    private Turret() { }





    //all the hardware goes here
    private final MotorEx lunchMotor= new MotorEx("lunchMotor").brakeMode();
    /*
    luncher angle(horizontal): 65  degrees

    Gear Ratio	19.2:1
    Encoder Resolution	537.7 PPR at the Output Shaft

     */
    private final MotorEx rotateMotor= new MotorEx("rotateMotor").brakeMode();
    /*
    13.7 : 1 Ratio, 435 RPM
    Encoder Resolution	384.5 PPR at the Output Shaft

    sku:5203-2402-0014 https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-6mm-d-shaft-435-rpm-36mm-gearbox-3-3-5v-encoder/

    24:135 (gear ratio for the turning plate)
    217.572 mm --> diameter for the plate --> plate Circumference:68.3523 cm

    Total Gear Ratio:
    13.7 * 135/24 = 77.0625

    total counts per plate revolution:
    77.0625 * 384.5 = 29630.53125 counts/plate rev

    __distance moved per encoder count__
    =68.3523/29630.53125 = 0.0023068267
     */

        double kp = 0.0001;
        double ki = 0.001;
        double kd = 0;
        double kf = 0;
        double maxPower = .1;
      private  double lastPosition=0.0;
      public final LimelightProcessing limelightProcessing = new LimelightProcessing();
    private final ControlSystem controlSystemTurret = ControlSystem.builder()
                .posPid(0.001, 0.6, 0.0009)
                .basicFF(0.00043)
                .build();
    private ControlSystem controlSystemRotate  = ControlSystem.builder()
                .posPid(kp, ki, kd)
                .basicFF(kf)
                .build();
        ;


    private List<AprilTagDetection> detectedTags = new ArrayList<>();
    private Telemetry telemetry;




    public double getRotateMotorPosition(){
        return rotateMotor.getCurrentPosition();
    }
    public double getLastPosition(){
        return lastPosition;
    }

    /*

    for webcam to work, copy from AprilTagWebCam.java

     */









    public double calculatePosition(){
        if(!limelightProcessing.processTargets().isEmpty()) {
            double angle = limelightProcessing.getTargetInfoByID(21).getTargetX();

            // DPE (Distance Per Encoder count) = 0.0023068267 cm/encoder count
            // This converts the arc length in cm to encoder counts for motor movement
          //  return arcLength / distancePerImpulseForRotation;
            return -angle/distancePerImpulseForRotation;
        }else{
            //return Math.PI*rotationDiameter/distancePerImpulseForRotation; //impulse needed to make a full rotation
            return 0;
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
    public double calculateLunchStrength(){
        double distance = 5;//aprilTagWebCam.getTagBySpecificID(21).ftcPose.range;
        double heightDifference =limelightProcessing.getTargetInfoByID(21).getTargetY() + 5;
        double tanTheta = Math.tan(Math.toRadians(detectedTags.get(21).ftcPose.bearing));
        double cosTheta = Math.cos(Math.toRadians(detectedTags.get(21).ftcPose.bearing));
        double efficientCoefficient = 0.9;


        double initialSpeedNeeded = Math.sqrt(gravityAccalerationValue*Math.pow(distance,2)/(2*Math.pow(cosTheta,2)*(distance*tanTheta+ heightDifference)))/efficientCoefficient;
        return initialSpeedNeeded*flyWheelDiameter/motorShaftRadiusForLuncher/distanceperImulseForLunch;
    }


//        public void launch() {
//            if (!detectedTags.isEmpty()) {
//                double strength = calculateLunchStrength();
//                lunchMotor.setPower(controlSystemTurret.calculate(
//                        new KineticState(0, strength)
//                ));
//            }
//        }

        /*
        the side of a tile is 61, and the "castle" is 45 degree within a tile
         */


        public double calculteYPosition(){
         double y = limelightProcessing.getTargetInfoByID(21).getTargetY();
         return 366 - 30.5 - y; //suppose the coordinate system in in cm
        }

        public double getSpeedNeeded(){
            double velocity = calculateLunchStrength();
            return controlSystemTurret.calculate(
                    new KineticState(0,velocity)
            );
        }



        @Override
        public void initialize() {
        // initialization logic (runs on init)
            rotateMotor.setPower(0);
            lunchMotor.setPower(0);
            limelightProcessing.initLimelight();



    }



        // post-start logic (runs once when start is pressed)

        @Override
        public void periodic() {
            limelightProcessing.processTargets();
            // remove after tuning, no need to rebuild control system every loop
            controlSystemRotate = ControlSystem.builder()
                    .posPid(kp, ki, kd)
                    .basicFF(kf)
                    .build();
            if (!ActiveOpMode.isStarted()){
                limelightProcessing.processTargets();
                // add detected tags to telemetry
                rotateMotor.setPower(0);
                lunchMotor.setPower(0);
                return;
            }

        // periodic logic (runs every loop)






            if(calculatePosition()!=0) {
                double turretPosition=rotateMotor.getCurrentPosition()+calculatePosition();
                controlSystemRotate.setGoal(new KineticState(turretPosition, 50));
                lastPosition=turretPosition;
            }
            else{
                //controlSystemRotate.setGoal(new KineticState(lastPosition, 50));
                controlSystemRotate.setGoal(new KineticState(0.0, 50));

            }
            if (!detectedTags.isEmpty()) {
                double x =limelightProcessing.getTargetInfoByID(21).getTargetX();

            }
            else{
                double x = 0;
            }

            double power = controlSystemRotate.calculate(
                    new KineticState(rotateMotor.getCurrentPosition())
            );

            //clamp power to limit during testing
            if(power > maxPower){
                power = maxPower;
            }else if(power < -maxPower){
                power = -maxPower;
            }
            rotateMotor.setPower(power);//end of tracking logic





        }

        public void rebuildControlSystem(double p, double i, double d, double f, double power){
            this.kp = p;
            this.ki = i;
            this.kd = d;
            this.kf = f;
            this.maxPower = power;
        }

}



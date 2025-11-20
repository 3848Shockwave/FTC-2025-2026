    package org.firstinspires.ftc.teamcode.Subsystems;


    import static org.firstinspires.ftc.teamcode.Subsystems.TurretConstants.distancePerImpulseForRotation;
    import static org.firstinspires.ftc.teamcode.Subsystems.TurretConstants.rotationDiameter;


    import com.qualcomm.hardware.limelightvision.LLResult;
    import com.qualcomm.hardware.limelightvision.LLResultTypes;
    import com.qualcomm.hardware.limelightvision.Limelight3A;
    import com.qualcomm.hardware.limelightvision.LLStatus;

    import dev.nextftc.control.ControlSystem;
    import dev.nextftc.ftc.ActiveOpMode;

    import org.firstinspires.ftc.robotcore.external.Telemetry;
    import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
    import org.firstinspires.ftc.teamcode.Subsystems.Helpers.LimelightProcessing;
    import org.firstinspires.ftc.teamcode.Tests.AprilTagWebCam;

    import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


    import java.util.ArrayList;
    import java.util.List;

    import dev.nextftc.control.KineticState;
    import dev.nextftc.core.subsystems.Subsystem;
    import dev.nextftc.hardware.impl.MotorEx;



    public class Turret implements Subsystem {
    //declare for every subsystem
    public static final Turret INSTANCE = new Turret();
        private static double kp = 0.004;
        private static double kd = 0.00026;
        private static double ki = 0.004;
        private static double kf = 0.0000275;
        private static double maxPower = 1.0;
        public final LimelightProcessing limelightProcessing = new LimelightProcessing(); //Creates limelight processing object
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
    private final ControlSystem controlSystemTurret = ControlSystem.builder()
            .posPid(0.001, 0.6, 0.0009)
            .basicFF(0.00043)
            .build();
    private ControlSystem controlSystemRotate  = ControlSystem.builder()
                .posPid(0, 0, 0)
                .basicFF(0)
                .build();


        public void rebuildControlSystem(double p, double i, double d, double f){
            this.kp = p;
            this.ki = i;
            this.kd = d;
            this.kf = f;


        }

        private Limelight3A limelight;
        private LLResult latestResult;
        private Pose3D botpose;








//    public double calculatePosition(){
//        if(!detectedTags.isEmpty()) {
//            double angle = aprilTagWebCam.getTagBySpecificID(21).ftcPose.bearing;
//            double arcLength = angle * Math.PI *rotationDiameter/360;
//
//            // DPE (Distance Per Encoder count) = 0.0023068267 cm/encoder count
//            // This converts the arc length in cm to encoder counts for motor movement
//            return arcLength / distancePerImpulseForRotation;
//        }else{
//            //return Math.PI*rotationDiameter/distancePerImpulseForRotation; //impulse needed to make a full rotation
//            return 0;
//        }
//    }


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

//    public double calculateLunchStrength(){
//        double distance = aprilTagWebCam.getTagBySpecificID(21).ftcPose.range;
//        double heightDifference = aprilTagWebCam.getTagBySpecificID(21).ftcPose.z + 5;
//        double tanTheta = Math.tan(Math.toRadians(detectedTags.get(21).ftcPose.bearing));
//        double cosTheta = Math.cos(Math.toRadians(detectedTags.get(21).ftcPose.bearing));
//        double efficientCoefficient = 0.9;
//
//
//        double initialSpeedNeeded = Math.sqrt(gravityAccalerationValue*Math.pow(distance,2)/(2*Math.pow(cosTheta,2)*(distance*tanTheta+ heightDifference)))/efficientCoefficient;
//        return initialSpeedNeeded*flyWheelDiameter/motorShaftRadiusForLuncher/distanceperImulseForLunch;
//    }
//
//
//        public double getSpeedNeeded(){
//            double velocity = calculateLunchStrength();
//            return controlSystemTurret.calculate(
//                    new KineticState(0,velocity)
//            );
//        }



        @Override
        public void initialize() {
        // initialization logic (runs on init)
            rotateMotor.setPower(0);
            lunchMotor.setPower(0);

            limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);
            limelight.start();


            if (!ActiveOpMode.isStarted()) {
                rotateMotor.setPower(0);
                return;
            }

    }



        // post-start logic (runs once when start is pressed)

        @Override
        public void periodic() {
            //this will get the LLresult from the limelight every cycle
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                //robot pose is the relative position from robot to the april tag
                botpose = result.getBotpose();
                //all the april tags from the limelight
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();


                /*
                get data we need;
                bearing & range
                 */
                if (!fiducials.isEmpty()) {
                    LLResultTypes.FiducialResult tag = fiducials.get(0);

                    double bearingDeg = tag.getTargetXDegrees();

                    /*
                    Equal to the calculatePosition, and keep the april tag in the center
                     */

                    double targetPosition = bearingDeg / distancePerImpulseForRotation;
                    controlSystemRotate.setGoal(new KineticState(targetPosition, 50));
                    double turretPosition = targetPosition + rotateMotor.getCurrentPosition();

                    double power = controlSystemRotate.calculate(
                            new KineticState(turretPosition)
                    );

                    rotateMotor.setPower(power);
                }

            }
        }
    }



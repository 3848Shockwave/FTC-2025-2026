package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.core.widget.TintableCheckedTextView;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.ifElseCommand;

import java.time.Duration;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPositions;
import dev.nextftc.hardware.powerable.SetPower;

public class Sort implements Subsystem {
    public static final Sort INSTANCE = new Sort();
    private final MotorEx turningPlate = new MotorEx("spindexMotor").brakeMode();
    MotorEx intake = new MotorEx("intake").brakeMode();
    //1,425.1 at output, 2:1 gear ratio, thus 2850.2 at motor shaft
    private final double ticksPerSlot = (2850.2) / 3;
    public Command pushBall = null;
    public Command backPosition = null;
    public ifElseCommand pushBallAndBack = null;
    public LambdaCommand positiveIntake = null;
    public LambdaCommand negativeIntake = null;
    public Command cycleLeft = null;
    public Command cycleRight = null;
    public Command loadGreen = null;
    public Command loadPurp = null;
    public Command shootGreen = null;
    public Command shootPurp = null;
    public Command overRideShoot = null;
    public Command cycleLeftAuto = null;
    boolean stopCommand = false;
//    public boolean intakeOn = true;
//    public Command switchIntake = null;

    HardwareMap hardwareMap;
    Telemetry telemetry;
    DigitalChannel limitSwitch;
    ServoEx servoLeft;

    // 5203-2402-0051, 50.9:1, 8mm(D) shaft, 117rpm, 1,425.1 PPR, 1:2 gear ratio. --> 72637.59 impulses per rotation
    //however, with the gear structure, the palate make a full turn every 145075.18 impulses, since there are 3 circle
    //every position should take exactly 48358 pulse
    ServoEx servoRight;
    ColorSensor colorSensorL1;
    ColorSensor colorSensorL2;
    ColorSensor colorSensorR1;
    ColorSensor colorSensorR2;

    public enum Color {
        GREEN, PURPLE, EMPTY;
    }
    private int tolerance = 0;
    private Color[] colorArray = {Color.EMPTY, Color.EMPTY, Color.EMPTY};

    double maxPowerSpindex =.75;
    private double kp = 0.00;
    private double ki = 0.0;
    private double kd = 0.0;
    private double kf = 0.0;
    private ElapsedTime timer = new ElapsedTime();
    private ControlSystem controlSystemSpindex = null;
    private ControlSystem controlSystem;
    private TelemetryManager telemetryManager;
    private double targetPosition = 0;
    private double powerToMove;
    private boolean intakeOn = true;
    public InstantCommand stopIntake = new InstantCommand(() -> {
        intakeOn = false;

    });
    public SequentialGroup tripleLaunch = null;


    private  Sort() {
    }


    //  public final Command nextBall = new RunToPosition(controlSystem, 48358).requires(this).named("nextBall");


//    private String detectLeftColor() {
//        String color = detectColor(colorSensorLeft1);
//        if ("UNKNOWN".equals(color)) {
//            color = detectColor(colorSensorLeft2);
//        }
//        return color;
//    }

    public double getCurrentPosition() {
        return turningPlate.getCurrentPosition();
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public double getPowerToMove() {
        return powerToMove;
    }

    public boolean getSpinLimitSwitchStatus() {
        boolean isPressed = limitSwitch.getState(); // Assuming active low
        return isPressed;
    }

   public String getColorArray() {
       return colorArray[0] + ", " + colorArray[1] + ", " + colorArray[2];
   }
    public void resetSpindexPosition() {
        turningPlate.setCurrentPosition(0.0);
    }

    // Method to detecprivate String detectRightColor() {

    ////        String color = detectColor(colorSensorRight1);
    ////        if ("UNKNOWN".equals(color)) {
    ////            color = detectColor(colorSensorRight2);
    ////        }
    ////        return color;
    ////    }t color with fallback for right group
//

    // Enhanced color detection method
//    private String detectColor(NormalizedColorSensor sensor) {
//        NormalizedRGBA colors = sensor.getNormalizedColors();
//
//        // Add threshold to avoid false readings
//        double threshold = 0.05; // Minimum intensity to consider a color
//
//        if (colors.red < threshold && colors.green < threshold && colors.blue < threshold) {
//            return "UNKNOWN"; // No significant color detected
//        }
//
//        if (colors.red > colors.blue && colors.red > colors.green) {
//            return "RED";
//        } else if (colors.blue > colors.red && colors.blue > colors.green) {
//            return "BLUE";
//        } else if (colors.green > colors.red && colors.green > colors.blue) {
//            return "GREEN";
//        } else {
//            return "UNKNOWN";
//        }
//    }
    public void checkColors() {
        int greenNumL = (colorSensorL1.green() + colorSensorL2.green()) / 2;
        int blueNumL = (colorSensorL1.blue() + colorSensorL2.blue()) / 2;
        int greenNumR = (colorSensorR1.green() + colorSensorR2.green()) / 2;
        int blueNumR = (colorSensorR1.blue() + colorSensorR2.blue()) / 2;

        if (greenNumR > 100 &&  blueNumR > 100){
            if (greenNumR > blueNumR){
                colorArray[0] = Color.GREEN;
            }
            else{
                colorArray[0] = Color.PURPLE;
            }
        }
        else{
            colorArray[0] = Color.EMPTY;
        }
        if(greenNumL > 100 && blueNumL > 100) {
            if (greenNumL > blueNumL) {
                colorArray[1] = Color.GREEN;
            } else {
                colorArray[1] = Color.PURPLE;
            }
        }
        else{
            colorArray[1] = Color.EMPTY;
        }

    }

    @Override
    public void initialize() {
        hardwareMap = ActiveOpMode.hardwareMap();
        telemetry = ActiveOpMode.telemetry();
        limitSwitch = hardwareMap.get(DigitalChannel.class, "spinLimitSwitch");
        Servo sLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoLeft = new ServoEx(sLeft);
        Servo sRight = hardwareMap.get(Servo.class, "servoRight");
        servoRight = new ServoEx(sRight);
        colorSensorL1 = hardwareMap.get(ColorSensor.class, "colorSensorL1");
        colorSensorL2 = hardwareMap.get(ColorSensor.class, "colorSensorL2");
        colorSensorR1 = hardwareMap.get(ColorSensor.class, "colorSensorR1");
        colorSensorR2 = hardwareMap.get(ColorSensor.class, "colorSensorR2");



        pushBall = new SetPositions(
                servoLeft.to(-1.0),
                servoRight.to(1.0)
        ).requires(this);


        backPosition = new SetPositions(
                servoLeft.to(1.0),
                servoRight.to(-1.0)
        ).requires(this);

        cycleLeft = new LambdaCommand()
                .setStart(() -> {

                    timer.reset();
                    timer.startTime();
                    controlSystemSpindex = ControlSystem.builder()
                            .posPid(0.01265, ki, 0.000012)
                            .basicFF(0.0001)
                            .build();
                    double goalPosition = targetPosition - ticksPerSlot;
                    targetPosition = goalPosition;
                    controlSystemSpindex.setGoal(new KineticState(goalPosition, 20));

                })
                .setUpdate(() -> {


                    powerToMove = controlSystemSpindex.calculate(
                            new KineticState(turningPlate.getCurrentPosition(), turningPlate.getVelocity())
                    );
                    if (powerToMove > maxPowerSpindex) {
                        powerToMove = maxPowerSpindex;
                    } else if (powerToMove < -maxPowerSpindex) {
                        powerToMove = -maxPowerSpindex;
                    }
                       turningPlate.setPower(powerToMove);

                })
                .setIsDone(() ->
                        limitSwitch.getState()&&
                        (controlSystemSpindex.isWithinTolerance(new KineticState(3)))                                                                                             )
                .setStop(interrupted -> {
                    turningPlate.setPower(0);
                    colorArray[2] = colorArray[0];
                    checkColors();
                    telemetryManager.addData("color array: ", getColorArray());
                    timer.reset();
                })
                .named("cycleLeft");
        cycleLeftAuto= new LambdaCommand().setStart(() -> {

                    timer.reset();
                    timer.startTime();
                    controlSystemSpindex = ControlSystem.builder()
                            .posPid(0.01265, ki, 0.000012)
                            .basicFF(0.0001)
                            .build();
                    double goalPosition = targetPosition - ticksPerSlot;
                    targetPosition = goalPosition;
                    controlSystemSpindex.setGoal(new KineticState(goalPosition, 20));
                })
                .setUpdate(() -> {


                    powerToMove = controlSystemSpindex.calculate(
                            new KineticState(turningPlate.getCurrentPosition(), turningPlate.getVelocity())
                    );
                    if (powerToMove > maxPowerSpindex) {
                        powerToMove = maxPowerSpindex;
                    } else if (powerToMove < -maxPowerSpindex) {
                        powerToMove = -maxPowerSpindex;
                    }
                    turningPlate.setPower(powerToMove);

                })
                .setIsDone(() ->
                        limitSwitch.getState()&&
                              (controlSystemSpindex.isWithinTolerance(new KineticState(10)))                                                                                             )
                .setStop(interrupted -> {
                    turningPlate.setPower(0);
                    timer.reset();
                }).setInterruptible(true).setName("cycleLeftAuto");

        cycleRight = new LambdaCommand()
                .setStart(() -> {

                    controlSystemSpindex = ControlSystem.builder()
                            .posPid(0.015, ki, 0.000012)
                            .basicFF(kf)
                            .build();
                    double goalPosition = targetPosition + ticksPerSlot;
                    targetPosition = goalPosition;
                    controlSystemSpindex.setGoal(new KineticState(goalPosition, 20));


                })
                .setUpdate(() -> {
                    powerToMove = controlSystemSpindex.calculate(
                            turningPlate.getState()
                    );
                    if (powerToMove > maxPowerSpindex) {
                        powerToMove = maxPowerSpindex;
                    } else if (powerToMove < -maxPowerSpindex) {
                        powerToMove = -maxPowerSpindex;
                    }
                    turningPlate.setPower(powerToMove);
                })
                .setIsDone(() ->
                        limitSwitch.getState()&&
                                (controlSystemSpindex.isWithinTolerance(new KineticState(3))))
                        .setStop(interrupted -> {
                    turningPlate.setPower(0);
                    colorArray[2] = colorArray[1];
                    checkColors();
                    telemetryManager.addData("color array: ", getColorArray());
                    timer.reset();
                })
                .requires(this)
                .named("cycleRight");

        pushBallAndBack = new ifElseCommand(
                () -> limitSwitch.getState(),
                new SetPositions(
                        servoLeft.to(-1.0),
                        servoRight.to(1.0)
                ).thenWait(0.45).then(new SetPositions(
                        servoLeft.to(1.0),
                        servoRight.to(-1.0)
                )),new SetPositions(
                servoLeft.to(-1.0),
                servoRight.to(1.0)
                ).thenWait(0.45).then(new SetPositions(
                servoLeft.to(1.0),
                servoRight.to(-1.0)
        )).requires(turningPlate.getVelocity()<5.0)


        );
        overRideShoot = new InstantCommand(()->{
            new SetPositions(
                    servoLeft.to(-1.0),
                    servoRight.to(1.0)
            ).thenWait(0.45).then(new SetPositions(
                    servoLeft.to(1.0),
                    servoRight.to(-1.0)
            ));
        }).setName("overRideShoot");

        positiveIntake= new LambdaCommand().setStart(()-> {
                    intake.setPower(1.0);
                }
        ).setInterruptible(true).setStop(interrupted->{
            intake.setPower(0.0);
        }).requires(intakeOn,this);
        negativeIntake= new LambdaCommand().setStart(()-> {
                    intake.setPower(-1.0);
                }
        ).setInterruptible(true).setStop(interrupted->{
            intake.setPower(0.0);
        }).requires(intakeOn,this);
        tripleLaunch = new SequentialGroup(
                pushBallAndBack,
                new Delay(.45),
                cycleLeftAuto.endAfter(.9),
                pushBallAndBack,
                new Delay(.45),
                cycleLeftAuto.endAfter(.9),
                pushBallAndBack.thenWait(.2)
        );


// Load Green
        loadGreen = new LambdaCommand()
                .setStart(() -> {



                    if (colorArray[0] == Color.GREEN) {

                        cycleLeft.schedule();
                    } else if (colorArray[1] == Color.GREEN) {
                        cycleRight.schedule();
                    } else {
                        return;
                    }

                })
                .setName("loadGreen")
                .requires(this);

// Load Purple
        loadPurp = new LambdaCommand()
                .setStart(() -> {


                    if (colorArray[0] == Color.PURPLE) {
                        cycleLeft.schedule();
                    } else if (colorArray[1] == Color.PURPLE) {
                        cycleRight.schedule();
                    } else {
                       return;
                    }

                })
                .setName("loadPurp")
                .requires(this);


// Shooting commands
//shootGreen = new LambdaCommand()
//        .setStart(() -> {
//            if (colorArray[2] == Color.GREEN) {
//                pushBallAndBack.schedule();
//            }
//        })
//        .setName("shootGreen")
//        .requires(this);
//
//shootPurp = new LambdaCommand()
//        .setStart(() -> {
//            if (colorArray[2] == Color.PURPLE) {
//                pushBallAndBack.schedule();
//            }
//        })
//        .setName("shootPurple")
//        .requires(this);








//        Button rightTriggerLoad = range(() -> ActiveOpMode.gamepad1().right_trigger)
//                .greaterThan(0.3)
//                .whenBecomesTrue(() -> {
//                    if (colorArray[0] == Color.GREEN){
//                        //cycle right
//
//
//                    }
//                    else if (colorArray[1] == Color.GREEN){
//                        //move motor left
//
//                    }
//                    else{
//
//
//                    }
//                });
//        Button rightTriggerShoot = range(() -> ActiveOpMode.gamepad1().right_trigger)
//                .greaterThan(0.95)
//                .whenBecomesTrue(() -> {
//
//
//                });
//
//
//        Button leftTriggerLoad = range(() -> ActiveOpMode.gamepad1() .left_trigger)
//                .greaterThan(0.3)
//                .whenBecomesTrue(() -> {
//
//
//                });
//
//
//        Button leftTriggetShoot = range(() -> ActiveOpMode.gamepad1().left_trigger)
//                .greaterThan(0.95)
//                .whenBecomesTrue(() -> {
//
//
//                });


        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        limitSwitch.setState(false);
        //   initialization logic (runs on init)
//        controlSystem = controlSystem.builder()
//                .posPid(0.01,0.6,0.009)
//                .basicFF(0.0005)
//                .build();


    }

    @Override
    public void periodic() {
        checkColors();
    }

//    public void rebuildControlSystem(double p, double i, double d, double f, double power) {
//        kp = p;
//        ki = i;
//        kd = d;
//        kf = f;
//        maxPowerSpindex = power;
//    }

}

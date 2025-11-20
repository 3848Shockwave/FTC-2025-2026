package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.ifElseCommand;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPositions;

public class Sort implements Subsystem {
    public static final Sort INSTANCE = new Sort();
    private final MotorEx turningPlate = new MotorEx("spindexMotor").brakeMode();
    //1,425.1 at output, 2:1 gear ratio, thus 2850.2 at motor shaft
    private final double ticksPerSlot = 2850.2 / 3;
    public Command pushBall = null;
    public Command backPosition = null;
    public ifElseCommand pushBallAndBack = null;
    public Command cycleLeft = null;
    public Command cycleRight = null;
    public Command loadGreen = null;
    public Command loadPurp = null;
    public Command shootGreen = null;
    public Command shootPurp = null;
    boolean stopCommand = false;
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
    private double kp = 0;
    private double ki = 0.00;
    private double kd = 0;
    private double kf = 0;
    private final ControlSystem controlSystemSpindex = ControlSystem.builder()
            .posPid(1, 0.0, 0.0000)
            .basicFF(0.0000)
            .build();
    private ControlSystem controlSystem;
    private TelemetryManager telemetryManager;
    private double targetPosition = 0;
    private double powerToMove;



    private Sort() {
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
                    double goalPosition = turningPlate.getCurrentPosition() - ticksPerSlot;
                    targetPosition = goalPosition;
                    controlSystemSpindex.setGoal(new KineticState(goalPosition, 50));

                })
                .setUpdate(() -> {
                      powerToMove = controlSystemSpindex.calculate(
                            new KineticState(turningPlate.getCurrentPosition())
                    );
                    turningPlate.setPower(powerToMove);
                })
                .setIsDone(() -> Math.abs(turningPlate.getCurrentPosition() - targetPosition) < 2                                                                                                   )
                .setStop(interrupted -> {
                    turningPlate.setPower(0);
                })
                .requires(this)
                .named("cycleLeft");

        cycleRight = new LambdaCommand()
                .setStart(() -> {
                    double goalPosition = turningPlate.getCurrentPosition() + ticksPerSlot;
                    targetPosition = goalPosition;
                    controlSystemSpindex.setGoal(new KineticState(goalPosition, 50));

                })
                .setUpdate(() -> {
                    powerToMove = controlSystemSpindex.calculate(
                            new KineticState(turningPlate.getCurrentPosition())
                    );
                    turningPlate.setPower(powerToMove);
                })
                .setIsDone(() -> Math.abs(turningPlate.getCurrentPosition() - targetPosition) < 2)
                .setStop(interrupted -> {
                    turningPlate.setPower(0);
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
                ))
        );

        loadGreen = new LambdaCommand()
                .setStart(()->{
                    if (colorArray[0] == Color.GREEN) {
                        cycleLeft.run();
                    }
                    else if (colorArray[1] == Color.GREEN){
                        cycleRight.run();
                        }

                })
                .setUpdate(()->{




                        })
                .setIsDone(()->{
                    return null;
                        })
                .setStop(interrupted->{

                }).setName("loadgreen").requires(this);

        loadPurp = new LambdaCommand()
                .setStart(()->{
                    if (colorArray[0] == Color.PURPLE) {
                        cycleLeft.run();
                    }
                    else if (colorArray[1] == Color.PURPLE){
                        cycleRight.run();
                    }

                })
                .setUpdate(()->{

                })
                .setIsDone(()->{
                    return null;
                })
                .setStop(interrupted->{

                }).setName("loadpurp").requires(this);

        shootGreen = new LambdaCommand()
                .setStart(()->{


                })
                .setUpdate(()->{

                })
                .setIsDone(()->{
                    return null;
                })
                .setStop(interrupted->{

                }).setName("shootgreen").requires(this);

        shootPurp = new LambdaCommand()
                .setStart(()->{
                    if (colorArray[0] == Color.GREEN) {
                        cycleLeft.run();
                    }
                    else if (colorArray[1] == Color.GREEN){
                        cycleRight.run();
                    }

                })
                .setUpdate(()->{

                })
                .setIsDone(()->{
                    return null;
                })
                .setStop(interrupted->{

                }).setName("shootpurp").requires(this);

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

    }

    @Override
    public void periodic() {



    }

    public void rebuildControlSystem(double p, double i, double d, double f) {
        this.kp = p;
        this.ki = i;
        this.kd = d;
        this.kf = f;
    }

}

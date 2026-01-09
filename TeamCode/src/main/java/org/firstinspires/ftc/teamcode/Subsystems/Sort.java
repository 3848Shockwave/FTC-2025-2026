package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.ELCEncoderV2;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.ifElseCommand;

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

public class Sort implements Subsystem {
    public static final Sort INSTANCE = new Sort();

    MotorEx intake = new MotorEx("intake").brakeMode();
    //1,425.1 at output, 2:1 gear ratio, thus 2850.2 at motor shaft

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

    boolean stopCommand = false;

    HardwareMap hardwareMap;
    Telemetry telemetry;
    DigitalChannel limitSwitch;
    ServoEx servoLeft;
     ELCEncoderV2 spindexEncoder;
     CRServo spindexRight;
     CRServo spindexLeft;

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

    public static double kp = 0.01;
    public static double ki = 0.00;
    public static double kd = 0.00001;
    double goal = 260;
    private ElapsedTime timer = new ElapsedTime();

    private TelemetryManager telemetryManager;
    private double targetPosition = 0;
    private double powerToMove;
    private boolean intakeOn = true;
    ControlSystem spindexControl = null;
    public InstantCommand stopIntake = new InstantCommand(() -> {
        intakeOn = false;

    });
    public SequentialGroup tripleLaunch = null;


    private  Sort() {
    }

    public double getCurrentPosition() {
        return spindexEncoder.getTotalDegrees();
    }

    public double getTargetPosition() {
        return targetPosition;
    }


    public boolean getSpinLimitSwitchStatus() {
        boolean isPressed = limitSwitch.getState(); // Assuming active low
        return isPressed;
    }

    public String getColorArray() {
        return colorArray[0] + ", " + colorArray[1] + ", " + colorArray[2];
    }

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
        Servo sLeft = hardwareMap.get(Servo.class, "scissorLeft");
        servoLeft = new ServoEx(sLeft);
        Servo sRight = hardwareMap.get(Servo.class, "scissorRight");
        servoRight = new ServoEx(sRight);
        colorSensorL1 = hardwareMap.get(ColorSensor.class, "colorSensorL1");
        colorSensorL2 = hardwareMap.get(ColorSensor.class, "colorSensorL2");
        colorSensorR1 = hardwareMap.get(ColorSensor.class, "colorSensorR1");
        colorSensorR2 = hardwareMap.get(ColorSensor.class, "colorSensorR2");
        spindexEncoder = new ELCEncoderV2(hardwareMap, "spindexEncoder");
        spindexRight = hardwareMap.get(CRServo.class, "spindexRight");
        spindexLeft = hardwareMap.get(CRServo.class, "spindexLeft");
        spindexControl = dev.nextftc.control.ControlSystem.builder()
                .posPid(kp,ki,kd)
                .posFilter(filter->filter.lowPass(.3))
                .build();
        spindexControl.setGoal( new KineticState(goal));
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
                    goal+=120;
                    spindexControl.setGoal( new KineticState(goal));

                })
                .setUpdate(() -> {
                    spindexEncoder.updateRotations();
                    double servoPower = spindexControl.calculate(spindexEncoder.getState());
                    if(servoPower>0.3){
                        servoPower=0.3;
                    }
                    if(servoPower<-0.3){
                        servoPower=-0.3;
                    }
                    if(!spindexControl.isWithinTolerance(new KineticState(4))) {
                        spindexRight.setPower(servoPower);
                        spindexLeft.setPower(servoPower);
                    }
                })
                .setIsDone(()-> spindexControl.isWithinTolerance(new KineticState(4)))
                .named("cycleLeft");

        cycleRight = new LambdaCommand()
                .setStart(() -> {
                    goal-=120;
                    spindexControl.setGoal( new KineticState(goal));

                })
                .setUpdate(() -> {
                    spindexEncoder.updateRotations();
                    double servoPower = spindexControl.calculate(spindexEncoder.getState());
                    if(servoPower>0.3){
                        servoPower=0.3;
                    }
                    if(servoPower<-0.3){
                        servoPower=-0.3;
                    }
                    if(!spindexControl.isWithinTolerance(new KineticState(4))) {
                        spindexRight.setPower(servoPower);
                        spindexLeft.setPower(servoPower);
                    }
                })
                .setIsDone(()-> spindexControl.isWithinTolerance(new KineticState(4)))
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
        )).requires(spindexControl.isWithinTolerance(new KineticState(4)))
        );



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
                cycleLeft.endAfter(.9),
                pushBallAndBack,
                new Delay(.45),
                cycleLeft.endAfter(.9),
                pushBallAndBack.thenWait(.2)
        );


        loadGreen = new LambdaCommand()
                .setStart(()->{
                    if (colorArray[0] == Color.GREEN) {
                        cycleLeft.schedule();
                    }
                    else if (colorArray[1] == Color.GREEN){
                        cycleRight.schedule();
                    }
                    else {
                        return;
                    }
                })
                .setName("loadgreen").requires(this);

        loadPurp = new LambdaCommand()
                .setStart(()->{
                    if (colorArray[0] == Color.PURPLE) {
                        cycleLeft.schedule();
                    }
                    else if (colorArray[1] == Color.PURPLE){
                        cycleRight.schedule();
                    }
                    else {
                        return;
                    }
                })
                .setName("loadpurp").requires(this);


        shootGreen = new LambdaCommand()
                .setStart(()->{
                    if (colorArray[2] == Color.GREEN){
                        pushBallAndBack.run();
                    }
                }).setName("shootgreen").requires(this);

        shootPurp = new LambdaCommand()
                .setStart(()->{
                    if (colorArray[0] == Color.PURPLE) {
                        pushBallAndBack.run();
                    }
                    else if (colorArray[1] == Color.GREEN){
                        cycleRight.run();
                    }
                })
                .setName("shootpurp").requires(this);

        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        limitSwitch.setState(false);
    }

    @Override
    public void periodic() {
        checkColors();
    }
}
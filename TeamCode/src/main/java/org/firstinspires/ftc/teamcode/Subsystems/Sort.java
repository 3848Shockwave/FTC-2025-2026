package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.ELCEncoderV2;

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
    ELCEncoderV2 spindexEncoder = null;
    ServoEx spindexRight;
    ServoEx spindexLeft;

    ServoEx servoLeft;
    ServoEx servoRight;

    ColorSensor colorSensorL1, colorSensorL2;
    ColorSensor colorSensorR1, colorSensorR2;

    HardwareMap hardwareMap;
    Telemetry telemetry;

    private int currentIndex = 0;
    private double currentSpindexVelocity = 0.0;

    public enum Color {
        GREEN, PURPLE, EMPTY
    }

    private Color[] colorArray = {Color.EMPTY, Color.EMPTY, Color.EMPTY};
    private boolean intakeOn = true;

    public Command pushBall = null;
    public LambdaCommand pushBallAndBack = null;
    public LambdaCommand positiveIntake = null;
    public LambdaCommand negativeIntake = null;
    public InstantCommand cycleLeft;
    public InstantCommand cycleRight;
    public Command shootGreen = null;
    public Command shootPurp = null;
    public Command tripleLaunch = null;
    public Command waitForPosition = null;



    private double[] POSITIONS = {0.045, 0.51, 0.999};

    private double[] TARGET_DEGREES = {122, 238, 3.5};
    private double ALLOWED_ERROR = 5.0;

    public InstantCommand stopIntake = new InstantCommand(() -> {
        intakeOn = false;
    });

    private Sort() {}

    @Override
    public void initialize() {
        hardwareMap = ActiveOpMode.hardwareMap();
        telemetry = ActiveOpMode.telemetry();
        spindexEncoder = new ELCEncoderV2(ActiveOpMode.hardwareMap(),"spindexEncoder");

        servoLeft = new ServoEx(hardwareMap.get(Servo.class, "scissorLeft"));
        servoRight = new ServoEx(hardwareMap.get(Servo.class, "scissorRight"));

        colorSensorL1 = hardwareMap.get(ColorSensor.class, "colorSensorL1");
        colorSensorL2 = hardwareMap.get(ColorSensor.class, "colorSensorL2");
        colorSensorR1 = hardwareMap.get(ColorSensor.class, "colorSensorR1");
        colorSensorR2 = hardwareMap.get(ColorSensor.class, "colorSensorR2");

        spindexRight = new ServoEx(hardwareMap.get(Servo.class, "spindexRight"));
        spindexLeft = new ServoEx(hardwareMap.get(Servo.class, "spindexLeft"));

        // Fix: Use conditional logic inside execution, fix requires(this)
        pushBallAndBack = new LambdaCommand()
                .setStart(() -> {
                    if (Math.abs(spindexEncoder.computeVelocity(spindexEncoder.getTotalDegrees())) < 0.1) {
                        new SetPositions(
                                servoLeft.to(-1.0),
                                servoRight.to(1.0)
                        ).thenWait(0.4).then(new SetPositions(
                                servoLeft.to(1.0),
                                servoRight.to(-1.0)
                        )).schedule();
                    }
                })
                .named("pushBallAndBack")
                .requires(this);

        cycleLeft = (InstantCommand) new InstantCommand(() -> {
            moveSpindex(1);
        }).named("cycleLeft");

        cycleRight = (InstantCommand) new InstantCommand(() -> {
            moveSpindex(-1);
        }).named("cycleRight");

        // Fix: logic inside start, requires(this)
        positiveIntake = new LambdaCommand()
                .setStart(() -> {
                    if (intakeOn) intake.setPower(1.0);
                })
                .setInterruptible(true)
                .setStop(interrupted -> intake.setPower(0.0))
                .requires(this);

        negativeIntake = new LambdaCommand()
                .setStart(() -> {
                    if (intakeOn) intake.setPower(-1.0);
                })
                .setInterruptible(true)
                .setStop(interrupted -> intake.setPower(0.0))
                .requires(this);

        shootGreen = new InstantCommand(() -> {
            if (colorArray[2] == Color.GREEN) {
                new SequentialGroup(new Delay(1), pushBallAndBack).schedule();
            } else if (colorArray[1] == Color.GREEN) {
                new SequentialGroup(cycleLeft, new Delay(1), pushBallAndBack).schedule();
            } else if (colorArray[0] == Color.GREEN) {
                new SequentialGroup(cycleRight, new Delay(1), pushBallAndBack).schedule();
            }
        }).named("shootGreen");

        shootPurp = new InstantCommand(() -> {
            if (colorArray[2] == Color.PURPLE) {
                new SequentialGroup(new Delay(1), pushBallAndBack).schedule();
            } else if (colorArray[1] == Color.PURPLE) {
                new SequentialGroup(cycleLeft, new Delay(1), pushBallAndBack).schedule();
            } else if (colorArray[0] == Color.PURPLE) {
                new SequentialGroup(cycleRight, new Delay(1), pushBallAndBack).schedule();
            }
        }).named("shootPurp");

        waitForPosition = new InstantCommand(() -> {
            while (!waitToStable()) {
                telemetry.addData("Current Index", currentIndex);
                telemetry.addData("Current Velocity", currentSpindexVelocity);
                telemetry.update();
            }
        }).named("waitForPosition");



        tripleLaunch = new SequentialGroup(
                pushBallAndBack,
                new Delay(0.7),
                cycleLeft,
                waitForPosition,
                new Delay(0.7),
                cycleLeft,
                waitForPosition,
                pushBallAndBack
        ).named("tripleLaunch");

        updateServo();
    }

    private void moveSpindex(int direction) {
        currentIndex += direction;

        if (currentIndex > 2) {
            currentIndex = 0;
        } else if (currentIndex < 0) {
            currentIndex = 2;
        }

        Color temp;
        if (direction == 1) {
            temp = colorArray[2];
            colorArray[2] = colorArray[1];
            colorArray[1] = colorArray[0];
            colorArray[0] = temp;
        } else {
            temp = colorArray[0];
            colorArray[0] = colorArray[1];
            colorArray[1] = colorArray[2];
            colorArray[2] = temp;
        }
    }




    public boolean waitToStable(){
        return Math.abs(currentSpindexVelocity) == 0; // Note: currentSpindexVelocity needs to be updated in periodic
    }

    public void updateServo() {
        double targetPos = POSITIONS[currentIndex];
        spindexRight.setPosition(targetPos);
        spindexLeft.setPosition(targetPos);
    }

    public double getServoPosition() {
        return spindexRight.getPosition();
    }

    public int getCurrentIndex() {
        return currentIndex;
    }

    public double getCurrentDegree(){
        return spindexEncoder.getDegrees();
    }

    @Override
    public void periodic() {
        spindexEncoder.updateRotations();
        checkColors();
        updateServo();
        spindexEncoder.updateRotations();
        // Added to support waitToStable
        currentSpindexVelocity = spindexEncoder.computeVelocity(spindexEncoder.getTotalDegrees());
    }

    public void checkColors() {
        int greenNumL = (colorSensorL1.green() + colorSensorL2.green()) / 2;
        int blueNumL = (colorSensorL1.blue() + colorSensorL2.blue()) / 2;
        int greenNumR = (colorSensorR1.green() + colorSensorR2.green()) / 2;
        int blueNumR = (colorSensorR1.blue() + colorSensorR2.blue()) / 2;

        // Fix: Do not overwrite with EMPTY if sensors are blank (prevents clearing memory during rotation)
        if (greenNumR > 100 && blueNumR > 100) {
            colorArray[0] = (greenNumR > blueNumR) ? Color.GREEN : Color.PURPLE;
        }

        if (greenNumL > 100 && blueNumL > 100) {
            colorArray[1] = (greenNumL > blueNumL) ? Color.GREEN : Color.PURPLE;
        }
    }

    public Color[] getColorArray() {
        return colorArray;
    }
}
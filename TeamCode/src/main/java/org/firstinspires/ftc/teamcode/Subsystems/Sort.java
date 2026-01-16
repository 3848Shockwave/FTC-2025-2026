package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.ELCEncoderV2;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
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

    // === Constants ===
    // Positions defined based on Test code (A, C, B)
    // 0.0 -> Position A
    // 0.45 -> Position C
    // 0.92 -> Position B
    private double[] POSITIONS = {0.045, 0.51, 0.999};
    private int currentIndex = 0; // Current slot index (0, 1, or 2)

    private double currentSpindexVelocity = 0.0;
    boolean hasRunL = false;
    boolean hasRunR = false;
    private boolean secondPressSeen = false;


    public enum Color {
        GREEN, PURPLE, EMPTY
    }
    // Assuming index 0 is Right side, index 1 is Left side (Adjust based on physical mounting)
    private Color[] colorArray = {Color.EMPTY, Color.EMPTY, Color.EMPTY};
    private boolean intakeOn = true;

    // === Commands ===
    public Command pushBall = null;
    public LambdaCommand pushBallAndBack = null;
    public LambdaCommand positiveIntake = null;
    public LambdaCommand negativeIntake = null;

    public LambdaCommand cycleLeft ;
    public LambdaCommand cycleRight;
    public Command shootGreen = null;
    public Command shootPurp = null;
    public SequentialGroup tripleLaunch = null;
    private boolean spindexIsStable = false;
    private boolean running = false;
    RevTouchSensor touchSensor = null;

    public InstantCommand stopIntake = new InstantCommand(() -> {
        intakeOn = false;
    });

    private Sort() {
    }

    @Override

    public void initialize() {
        hardwareMap = ActiveOpMode.hardwareMap();
        telemetry = ActiveOpMode.telemetry();
        spindexEncoder = new ELCEncoderV2(ActiveOpMode.hardwareMap(),"spindexEncoder");
        touchSensor = ActiveOpMode.hardwareMap().get(RevTouchSensor.class, "touchSensor");
        servoLeft = new ServoEx(hardwareMap.get(Servo.class, "scissorLeft"));
        servoRight = new ServoEx(hardwareMap.get(Servo.class, "scissorRight"));

        colorSensorL1 = hardwareMap.get(ColorSensor.class, "colorSensorL1");
        colorSensorL2 = hardwareMap.get(ColorSensor.class, "colorSensorL2");
        colorSensorR1 = hardwareMap.get(ColorSensor.class, "colorSensorR1");
        colorSensorR2 = hardwareMap.get(ColorSensor.class, "colorSensorR2");

        spindexRight = new ServoEx(hardwareMap.get(Servo.class, "spindexRight"));
        spindexLeft = new ServoEx(hardwareMap.get(Servo.class, "spindexLeft"));


        pushBallAndBack = new LambdaCommand()
                .setStart(() -> {
                    // Reset internal latch
                    secondPressSeen = false;
                    running=false;
                })
                .setUpdate(() -> {
                    // Only execute push when spindex is stable
                    if (spindexIsStable && !secondPressSeen&&!running) {
                        new SetPositions(
                                servoLeft.to(-1.0),
                                servoRight.to(1.0)
                        )
                                .thenWait(0.4)
                                .then(new SetPositions(
                                        servoLeft.to(1.0),
                                        servoRight.to(-1.0)
                                ))
                                .schedule();
                        running = true;
                    }

                    // Detect the second press
                    if (touchSensor.isPressed()) {
                        secondPressSeen = true;
                    }
                })
                .setIsDone(() -> secondPressSeen)
                .named("pushBallAndBack");
        // Core Rotation Logic
        // Rotate Left (Index + 1)
        cycleLeft = new LambdaCommand()
                .setStart(()->{
                     hasRunL = false;
                 })
                .setUpdate(() -> {
                    if (!hasRunL && touchSensor.isPressed()) {
                        moveSpindex(1);
                        hasRunL = true;
                    }
                })
                .setIsDone(() -> hasRunL)
                .named("cycleLeft");

        cycleRight = new LambdaCommand()
                .setStart(()->{
                    hasRunR = false;
                })
                .setUpdate(() -> {
                    if (!hasRunR && touchSensor.isPressed()) {
                        moveSpindex(-1);
                        hasRunR = true;
                    }
                })
                .setIsDone(() -> hasRunR)
                .named("cycleRight");



        positiveIntake = new LambdaCommand()
                .setStart(() -> intake.setPower(1.0))
                .setInterruptible(true)
                .setStop(interrupted -> intake.setPower(0.0))
                .requires(intakeOn, this);

        negativeIntake = new LambdaCommand()
                .setStart(() -> intake.setPower(-1.0))
                .setInterruptible(true)
                .setStop(interrupted -> intake.setPower(0.0))
                .requires(intakeOn, this);


        shootGreen = new InstantCommand(() -> {
            if (colorArray[2] == Color.GREEN) {
                new SequentialGroup(
                        new Delay(1),
                        pushBallAndBack
                ).schedule();
            }
            else if (colorArray[1] == Color.GREEN) {
                new SequentialGroup(
                        cycleLeft,
                        new Delay(1),
                        pushBallAndBack
                ).schedule();
            }
            else if (colorArray[0] == Color.GREEN) {
                new SequentialGroup(
                        cycleRight,
                        new Delay(1),
                        pushBallAndBack
                ).schedule();
            }
        }).named("shootGreen");

        shootPurp = new InstantCommand(() -> {
            if (colorArray[2] == Color.PURPLE) {
                new SequentialGroup(
                        new Delay(1),
                        pushBallAndBack
                ).schedule();
            }
            else if (colorArray[1] == Color.PURPLE) {
                new SequentialGroup(
                        cycleLeft,
                        new Delay(1),
                        pushBallAndBack
                ).schedule();
            }
            else if (colorArray[0] == Color.PURPLE) {
                new SequentialGroup(
                        cycleRight,
                        new Delay(1),
                        pushBallAndBack
                ).schedule();
            }
        }).named("shootPurp");




        updateServo();


    }



     // direction 1 for clockwise (Next slot), -1 for counter-clockwise (Previous slot)
     private void moveSpindex(int direction) {
         currentIndex += direction;

         if (currentIndex > 2) {
             currentIndex = 0;
         } else if (currentIndex < 0) {
             currentIndex = 2;
         }

         //Memory Shifting (Virtual Rotation)
         Color temp;
         if (direction == 1) {
             // Rotating Left (1->2, 0->1, 2->0)
             temp = colorArray[2];
             colorArray[2] = colorArray[1];
             colorArray[1] = colorArray[0];
             colorArray[0] = temp;
         } else {
             // Rotating Right (0->2, 1->0, 2->1)
             temp = colorArray[0];
             colorArray[0] = colorArray[1];
             colorArray[1] = colorArray[2];
             colorArray[2] = temp;
         }

     }
    Command waitForStableCmd = new LambdaCommand()
            .setIsDone(this::waitToStable)
            .named("WaitForStable");
    public boolean waitToStable(){
        return Math.abs(currentSpindexVelocity) == 0;
    }

    public void updateServo() {
        double targetPos = POSITIONS[currentIndex];
        spindexRight.setPosition(targetPos);
        spindexLeft.setPosition(targetPos);
    }

    public double getServoPosition() {
        // Returns the last position set (e.g., 0.0, 0.45, or 0.92)
        return spindexRight.getPosition();
    }

    public int getCurrentIndex() {
        // Returns the logical slot number (0, 1, or 2)
        return currentIndex;
    }

    @Override
    public void periodic() {
        checkColors();
        updateServo();
        spindexEncoder.updateRotations();
        spindexIsStable = spindexEncoder.isStable(0.5, 3);
        ActiveOpMode.telemetry().addData("=====Sorting Data======", "");
        ActiveOpMode.telemetry().addData("SpindexStable", spindexIsStable);
        ActiveOpMode.telemetry().addData("Touchy", touchSensor.isPressed());
    }

    public void checkColors() {
        int greenNumL = (colorSensorL1.green() + colorSensorL2.green()) / 2;
        int blueNumL = (colorSensorL1.blue() + colorSensorL2.blue()) / 2;
        int greenNumR = (colorSensorR1.green() + colorSensorR2.green()) / 2;
        int blueNumR = (colorSensorR1.blue() + colorSensorR2.blue()) / 2;

        // Color determination logic
        if (greenNumR > 100 && blueNumR > 100      ) {
            colorArray[0] = (greenNumR > blueNumR) ? Color.GREEN : Color.PURPLE;
        } else {
            colorArray[0] = Color.EMPTY;
        }

        if (greenNumL > 100 && blueNumL > 100) {
            colorArray[1] = (greenNumL > blueNumL) ? Color.GREEN : Color.PURPLE;
        } else {
            colorArray[1] = Color.EMPTY;
        }
    }


    // Getters
    public Color[] getColorArray() {
        return colorArray;
    }
}
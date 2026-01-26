package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    MotorEx intake = new MotorEx("intakeMotor").brakeMode();
    ELCEncoderV2 spindexEncoder = null;
    ServoEx spindexRight;
    ServoEx spindexLeft;

    ServoEx servoLeft;
    ServoEx servoRight;

    ColorSensor colorSensorL1, colorSensorL2;
    ColorSensor colorSensorR1, colorSensorR2;
    ColorSensor colorSensorFront;

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
    boolean shootComplete = false;
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
    final ElapsedTime timer = new ElapsedTime();

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
        colorSensorFront = hardwareMap.get(ColorSensor.class, "colorSensorFront");

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
                                .thenWait(0.5)
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
            colorArray[2]= Color.EMPTY;
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
            colorArray[2]= Color.EMPTY;
        }).named("shootPurp");


    }
    public boolean getSecondPressSeen(){
        return secondPressSeen;}



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

    public boolean isSpindexStable(){
        return spindexIsStable;
    }
    public boolean isTouchPressed(){
        return touchSensor.isPressed();
    }

    public void updateServo() {
        double targetPos = POSITIONS[currentIndex];
        spindexRight.setPosition(targetPos);
        spindexLeft.setPosition(targetPos);

    }
    public void restartScissor() {
        servoLeft.setPosition(1.0);
        servoRight.setPosition(-1.0);
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
        spindexIsStable = spindexEncoder.isStable(.25, 4);
    }

    public void checkColors() {
        int greenNumL = (colorSensorL1.green() + colorSensorL2.green()) / 2;
        int blueNumL = (colorSensorL1.blue() + colorSensorL2.blue()) / 2;
        int greenNumR = (colorSensorR1.green() + colorSensorR2.green()) / 2;
        int blueNumR = (colorSensorR1.blue() + colorSensorR2.blue()) / 2;
        int greenNumF = colorSensorFront.green();
        int blueNumF = colorSensorFront.blue();
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
        if (greenNumF > 100 && blueNumF > 100) {
            colorArray[2] = (greenNumF > blueNumF) ? Color.GREEN : Color.PURPLE;
        } else {
            colorArray[2] = Color.EMPTY;
        }
    }


    // Getters
    public Color[] getColorArray() {
        return colorArray;
    }
    
   private enum State { CHECK, WAIT_STABLE, PUSH, WAIT_RETRACT, DONE }

    public Command shootNewGreen() {
        if (colorArray[0] == Color.GREEN || colorArray[1] == Color.GREEN || colorArray[2] == Color.GREEN) {
            final State[] state = { State.CHECK };
            final int[] targetIndex = { -1 };

            final double retractDelay = 0.6;

            return new LambdaCommand()
                    .setStart(() -> {
                        shootComplete = false;
                        state[0] = State.CHECK;
                        targetIndex[0] = -1;
                        timer.reset();
                    })
                    .setUpdate(() -> {
                        switch (state[0]) {
                            case CHECK:
                                if (colorArray[2] == Color.GREEN) {
                                    targetIndex[0] = 2;
                                    state[0] = State.PUSH;
                                } else if (colorArray[1] == Color.GREEN) {
                                    targetIndex[0] = 1;
                                    cycleLeft.schedule();
                                    state[0] = State.WAIT_STABLE;
                                } else if (colorArray[0] == Color.GREEN) {
                                    targetIndex[0] = 0;
                                    cycleRight.schedule();
                                    state[0] = State.WAIT_STABLE;
                                } else {
                                    state[0] = State.DONE;
                                }
                                break;
                            case WAIT_STABLE:
                                if (Sort.INSTANCE.spindexIsStable) {
                                    // small settle
                                    timer.reset();
                                    state[0] = State.PUSH;
                                }
                                break;
                            case PUSH:
                                // schedule the push/retract sequence
                                new SetPositions(
                                        servoLeft.to(-1.0),
                                        servoRight.to(1.0)
                                )
                                        .thenWait(0.35)
                                        .then(new SetPositions(
                                                servoLeft.to(1.0),
                                                servoRight.to(-1.0)
                                        ))
                                        .schedule();
                                timer.reset();
                                state[0] = State.WAIT_RETRACT;
                                break;
                            case WAIT_RETRACT:
                                if (timer.seconds() >= retractDelay) {
                                    colorArray[2] = Color.EMPTY;
                                    shootComplete = true;
                                    state[0] = State.DONE;
                                }
                                break;
                            case DONE:
                                // no-op
                                break;
                        }
                    })
                    .setIsDone(() -> state[0] == State.DONE).setInterruptible(true)
                    .named("shootNewGreen");
        }
        return new InstantCommand(() -> {});
    }

    public Command shootNewPurp() {
        if (colorArray[0] == Color.PURPLE || colorArray[1] == Color.PURPLE || colorArray[2] == Color.PURPLE) {
            final State[] state = { State.CHECK };
            final int[] targetIndex = { -1 };
            final double retractDelay = 0.6;

            return new LambdaCommand()
                    .setStart(() -> {
                        shootComplete = false;
                        state[0] = State.CHECK;
                        targetIndex[0] = -1;
                        timer.reset();
                    })
                    .setUpdate(() -> {
                        switch (state[0]) {
                            case CHECK:
                                if (colorArray[2] == Color.PURPLE) {
                                    targetIndex[0] = 2;
                                    state[0] = State.PUSH;
                                } else if (colorArray[1] == Color.PURPLE) {
                                    targetIndex[0] = 1;
                                    cycleLeft.schedule();
                                    state[0] = State.WAIT_STABLE;
                                } else if (colorArray[0] == Color.PURPLE) {
                                    targetIndex[0] = 0;
                                    cycleRight.schedule();
                                    state[0] = State.WAIT_STABLE;
                                } else {
                                    state[0] = State.DONE;
                                }
                                break;
                            case WAIT_STABLE:
                                if (Sort.INSTANCE.spindexIsStable) {
                                    timer.reset();
                                    state[0] = State.PUSH;
                                }
                                break;
                            case PUSH:
                                new SetPositions(
                                        servoLeft.to(-1.0),
                                        servoRight.to(1.0)
                                )
                                        .thenWait(0.25)
                                        .then(new SetPositions(
                                                servoLeft.to(1.0),
                                                servoRight.to(-1.0)
                                        ))
                                        .schedule();
                                timer.reset();
                                state[0] = State.WAIT_RETRACT;
                                break;
                            case WAIT_RETRACT:
                                if (timer.seconds() >= retractDelay) {
                                    colorArray[2] = Color.EMPTY;
                                    shootComplete = true;
                                    state[0] = State.DONE;
                                }
                                break;
                            case DONE:
                                // no-op
                                break;
                        }
                    })
                    .setIsDone(() -> state[0] == State.DONE).setInterruptible(true)
                    .named("shootNewPurp");
        }
        return new InstantCommand(() -> {});
    }

    public Command shootClosestBall() {
        if (colorArray[2] != Color.EMPTY || colorArray[1] != Color.EMPTY || colorArray[0] != Color.EMPTY) {
            final State[] state = { State.CHECK };
            final int[] targetIndex = { -1 };

            final double retractDelay = 0.6;

            return new LambdaCommand()
                    .setStart(() -> {
                        state[0] = State.CHECK;
                        targetIndex[0] = -1;
                        timer.reset();
                    })
                    .setUpdate(() -> {
                        switch (state[0]) {
                            case CHECK:
                                if (colorArray[2] != Color.EMPTY) {
                                    targetIndex[0] = 2;
                                    state[0] = State.PUSH;
                                } else if (colorArray[1] != Color.EMPTY) {
                                    targetIndex[0] = 1;
                                    cycleLeft.schedule();
                                    state[0] = State.WAIT_STABLE;
                                } else if (colorArray[0] != Color.EMPTY) {
                                    targetIndex[0] = 0;
                                    cycleRight.schedule();
                                    state[0] = State.WAIT_STABLE;
                                } else {
                                    state[0] = State.DONE;
                                }
                                break;
                            case WAIT_STABLE:
                                if (Sort.INSTANCE.spindexIsStable) {
                                    timer.reset();
                                    state[0] = State.PUSH;
                                }
                                break;
                            case PUSH:
                                new SetPositions(
                                        servoLeft.to(-1.0),
                                        servoRight.to(1.0)
                                )
                                        .thenWait(0.35)
                                        .then(new SetPositions(
                                                servoLeft.to(1.0),
                                                servoRight.to(-1.0)
                                        ))
                                        .schedule();
                                timer.reset();
                                state[0] = State.WAIT_RETRACT;
                                break;
                            case WAIT_RETRACT:
                                if (timer.seconds() >= retractDelay) {
                                    colorArray[2] = Color.EMPTY;
                                    state[0] = State.DONE;
                                }
                                break;
                            case DONE:
                                // no-op
                                break;
                        }
                    })
                    .setIsDone(() -> state[0] == State.DONE).setInterruptible(true)
                    .named("shootClosestBall");
        }
        return new InstantCommand(() -> {}).named("shootClosestBall");
    }

}


package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.ELCEncoderV2;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

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
    private boolean spindexIsStable = false;
    private boolean running = false;
    RevTouchSensor touchSensor = null;

    public InstantCommand stopIntake = new InstantCommand(() -> {
        intakeOn = false;
    });

    // --- New internal sequence state machine fields ---
    private enum SequenceKind { NONE, SHOOT_GREEN, SHOOT_PURP, SHOOT_CLOSEST }
    private SequenceKind activeSequence = SequenceKind.NONE;
    private int seqTargetIndex = -1; // index of the ball we want to handle (0..2)
    private int seqPhase = 0;
    private long seqPhaseStartMs = 0;

    // push internal state (used both by pushBallAndBack if scheduled standalone or by sequence)
    private boolean pushRunning = false;
    private int pushStage = 0; // 0 none, 1 pushed, 2 retracted
    private long pushStartMs = 0;

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


        // Re-implement pushBallAndBack without scheduling other commands.
        // This LambdaCommand manages its own push/retract timing and completes when the second press is seen.
        pushBallAndBack = new LambdaCommand()
                .setStart(() -> {
                    secondPressSeen = false;
                    running = false;
                    pushRunning = false;
                    pushStage = 0;
                    pushStartMs = 0;
                })
                .setUpdate(() -> {
                    // If spindex is stable and we haven't started the push, begin it
                    if (spindexIsStable && !secondPressSeen && !pushRunning) {
                        // begin push: set servos to push positions
                        servoLeft.setPosition(-1.0);
                        servoRight.setPosition(1.0);
                        pushStartMs = System.currentTimeMillis();
                        pushRunning = true;
                        pushStage = 1;
                    }

                    // After 250 ms, retract
                    if (pushRunning && pushStage == 1) {
                        if (System.currentTimeMillis() - pushStartMs >= 250) {
                            servoLeft.setPosition(1.0);
                            servoRight.setPosition(-1.0);
                            pushStage = 2;
                        }
                    }

                    // Detect the second press at any time
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


        // Instead of creating SequentialGroup(...) and scheduling it here (which would call commands from within
        // a command), we start an internal sequence handled by periodic()/processSequence(). The InstantCommand
        // below merely requests the sequence start.
        shootGreen = new InstantCommand(() -> {
            // choose nearest GREEN slot (prefer index 2, then 1, then 0)
            int target = -1;
            if (colorArray[2] == Color.GREEN) target = 2;
            else if (colorArray[1] == Color.GREEN) target = 1;
            else if (colorArray[0] == Color.GREEN) target = 0;
            if (target != -1) {
                startSequence(SequenceKind.SHOOT_GREEN, target);
            }
            // keep original behavior of marking the current top as empty immediately
            if (target == 2) colorArray[2] = Color.EMPTY;
        }).named("shootGreen");

        shootPurp = new InstantCommand(() -> {
            int target = -1;
            if (colorArray[2] == Color.PURPLE) target = 2;
            else if (colorArray[1] == Color.PURPLE) target = 1;
            else if (colorArray[0] == Color.PURPLE) target = 0;
            if (target != -1) {
                startSequence(SequenceKind.SHOOT_PURP, target);
            }
            if (target == 2) colorArray[2] = Color.EMPTY;
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
        spindexIsStable = spindexEncoder.isStable(0.5, 3);
        ActiveOpMode.telemetry().addData("=====Sorting Data======", "");
        ActiveOpMode.telemetry().addData("SpindexStable", spindexIsStable);
        ActiveOpMode.telemetry().addData("Touchy", touchSensor.isPressed());

        // Run any active sequence state machine
        processSequence();
    }

    // Internal sequence processing: replicates the former SequentialGroup flows without scheduling commands.
    private void processSequence() {
        if (activeSequence == SequenceKind.NONE) return;
        long now = System.currentTimeMillis();

        switch (activeSequence) {
            case SHOOT_GREEN:
            case SHOOT_PURP: {
                // Behavior depends on seqTargetIndex
                if (seqPhase == 0) {
                    // initial phase before any action
                    if (seqTargetIndex == 2) {
                        // previously: Delay(1) then pushBallAndBack
                        seqPhaseStartMs = now;
                        seqPhase = 10; // wait delay then push
                    } else if (seqTargetIndex == 1) {
                        // previously: cycleLeft, Delay(1), pushBallAndBack (but cycleLeft waited for touch press)
                        seqPhase = 20; // wait for touch to cycle left
                    } else if (seqTargetIndex == 0) {
                        seqPhase = 30; // wait for touch to cycle right
                    }
                }

                // Target at top (2)
                if (seqPhase == 10) {
                    // wait 1 second then start push sequence
                    if (now - seqPhaseStartMs >= 1000) {
                        // start push sequence: ensure spindex is stable first
                        if (spindexIsStable) {
                            // begin push
                            servoLeft.setPosition(-1.0);
                            servoRight.setPosition(1.0);
                            pushStartMs = now;
                            pushRunning = true;
                            pushStage = 1;
                            seqPhase = 11;
                        }
                    }
                }
                if (seqPhase == 11) {
                    // handle push timing: retract after 250ms then wait for second press to finish
                    if (pushRunning && pushStage == 1) {
                        if (now - pushStartMs >= 250) {
                            servoLeft.setPosition(1.0);
                            servoRight.setPosition(-1.0);
                            pushStage = 2;
                        }
                    }
                    if (pushStage == 2 && touchSensor.isPressed()) {
                        // finish sequence
                        colorArray[2] = Color.EMPTY;
                        activeSequence = SequenceKind.NONE;
                        seqPhase = 0;
                        pushRunning = false;
                        pushStage = 0;
                        shootComplete = true;
                    }
                }

                // Target at index 1: need to wait for touch press to cycle left
                if (seqPhase == 20) {
                    if (touchSensor.isPressed()) {
                        moveSpindex(1);
                        // after moving, wait until spindex is stable then delay .1 then push
                        seqPhase = 21;
                        seqPhaseStartMs = now;
                    }
                }
                if (seqPhase == 21) {
                    if (spindexIsStable) {
                        // small delay 100ms then push
                        if (now - seqPhaseStartMs >= 100) {
                            // start push
                            servoLeft.setPosition(-1.0);
                            servoRight.setPosition(1.0);
                            pushStartMs = now;
                            pushRunning = true;
                            pushStage = 1;
                            seqPhase = 22;
                        }
                    }
                }
                if (seqPhase == 22) {
                    if (pushRunning && pushStage == 1) {
                        if (now - pushStartMs >= 250) {
                            servoLeft.setPosition(1.0);
                            servoRight.setPosition(-1.0);
                            pushStage = 2;
                        }
                    }
                    if (pushStage == 2 && touchSensor.isPressed()) {
                        colorArray[2] = Color.EMPTY;
                        activeSequence = SequenceKind.NONE;
                        seqPhase = 0;
                        pushRunning = false;
                        pushStage = 0;
                        shootComplete = true;
                    }
                }

                // Target at index 0: rotate right then same as above
                if (seqPhase == 30) {
                    if (touchSensor.isPressed()) {
                        moveSpindex(-1);
                        seqPhase = 31;
                        seqPhaseStartMs = now;
                    }
                }
                if (seqPhase == 31) {
                    if (spindexIsStable) {
                        if (now - seqPhaseStartMs >= 100) {
                            servoLeft.setPosition(-1.0);
                            servoRight.setPosition(1.0);
                            pushStartMs = now;
                            pushRunning = true;
                            pushStage = 1;
                            seqPhase = 32;
                        }
                    }
                }
                if (seqPhase == 32) {
                    if (pushRunning && pushStage == 1) {
                        if (now - pushStartMs >= 250) {
                            servoLeft.setPosition(1.0);
                            servoRight.setPosition(-1.0);
                            pushStage = 2;
                        }
                    }
                    if (pushStage == 2 && touchSensor.isPressed()) {
                        colorArray[2] = Color.EMPTY;
                        activeSequence = SequenceKind.NONE;
                        seqPhase = 0;
                        pushRunning = false;
                        pushStage = 0;
                        shootComplete = true;
                    }
                }

                break;
            }

            case SHOOT_CLOSEST: {
                // Similar to earlier shootClosestBall: prefer 2, then 1, then 0, but with different delay values
                if (seqPhase == 0) {
                    if (seqTargetIndex == 2) {
                        seqPhase = 100;
                        seqPhaseStartMs = now;
                    } else if (seqTargetIndex == 1) {
                        seqPhase = 200; // cycle left then short delay then push
                    } else if (seqTargetIndex == 0) {
                        seqPhase = 300; // cycle right then short delay then push
                    }
                }
                if (seqPhase == 100) {
                    // wait 500ms then push
                    if (now - seqPhaseStartMs >= 500) {
                        if (spindexIsStable) {
                            servoLeft.setPosition(-1.0);
                            servoRight.setPosition(1.0);
                            pushStartMs = now; pushRunning = true; pushStage = 1; seqPhase = 101;
                        }
                    }
                }
                if (seqPhase == 101) {
                    if (pushRunning && pushStage == 1 && now - pushStartMs >= 250) {
                        servoLeft.setPosition(1.0); servoRight.setPosition(-1.0); pushStage = 2;
                    }
                    if (pushStage == 2) {
                        colorArray[2] = Color.EMPTY;
                        activeSequence = SequenceKind.NONE; seqPhase = 0; pushRunning=false; pushStage=0;
                    }
                }

                if (seqPhase == 200) {
                    if (touchSensor.isPressed()) {
                        moveSpindex(1); seqPhase = 201; seqPhaseStartMs = now;
                    }
                }
                if (seqPhase == 201) {
                    if (now - seqPhaseStartMs >= 500 && spindexIsStable) {
                        // push
                        servoLeft.setPosition(-1.0); servoRight.setPosition(1.0);
                        pushStartMs = now; pushRunning = true; pushStage = 1; seqPhase = 202;
                    }
                }
                if (seqPhase == 202) {
                    if (pushRunning && pushStage == 1 && now - pushStartMs >= 250) {
                        servoLeft.setPosition(1.0); servoRight.setPosition(-1.0); pushStage = 2;
                    }
                    if (pushStage == 2) {
                        colorArray[2] = Color.EMPTY; activeSequence = SequenceKind.NONE; seqPhase=0; pushRunning=false; pushStage=0;
                    }
                }

                if (seqPhase == 300) {
                    if (touchSensor.isPressed()) {
                        moveSpindex(-1); seqPhase = 301; seqPhaseStartMs = now;
                    }
                }
                if (seqPhase == 301) {
                    if (now - seqPhaseStartMs >= 500 && spindexIsStable) {
                        servoLeft.setPosition(-1.0); servoRight.setPosition(1.0);
                        pushStartMs = now; pushRunning = true; pushStage = 1; seqPhase = 302;
                    }
                }
                if (seqPhase == 302) {
                    if (pushRunning && pushStage == 1 && now - pushStartMs >= 250) {
                        servoLeft.setPosition(1.0); servoRight.setPosition(-1.0); pushStage = 2;
                    }
                    if (pushStage == 2) {
                        colorArray[2] = Color.EMPTY; activeSequence = SequenceKind.NONE; seqPhase=0; pushRunning=false; pushStage=0;
                    }
                }

                break;
            }

        }
    }

    // Helper to start a sequence from commands. This avoids scheduling any commands inside another command.
    private void startSequence(SequenceKind kind, int targetIndex) {
        this.activeSequence = kind;
        this.seqTargetIndex = targetIndex;
        this.seqPhase = 0;
        this.seqPhaseStartMs = System.currentTimeMillis();
        this.pushRunning = false;
        this.pushStage = 0;
        this.pushStartMs = 0;
        this.shootComplete = false;
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
    public Command shootNewGreen() {
        if(colorArray[0]==Color.GREEN||colorArray[1]==Color.GREEN||colorArray[2]==Color.GREEN) {


            return new InstantCommand(() -> {
                // Start internal shoot green sequence (preserves previous behavior but without scheduling commands)
                int target = -1;
                if (colorArray[2] == Color.GREEN) target = 2;
                else if (colorArray[1] == Color.GREEN) target = 1;
                else if (colorArray[0] == Color.GREEN) target = 0;
                if (target != -1) startSequence(SequenceKind.SHOOT_GREEN, target);
            }).named("shootNewGreen");
        }
        return new InstantCommand(()->{});
    }

    public Command shootNewPurp() {
        if(colorArray[0]==Color.PURPLE||colorArray[1]==Color.PURPLE||colorArray[2]==Color.PURPLE) {
            return new InstantCommand(() -> {
                int target = -1;
                if (colorArray[2] == Color.PURPLE) target = 2;
                else if (colorArray[1] == Color.PURPLE) target = 1;
                else if (colorArray[0] == Color.PURPLE) target = 0;
                if (target != -1) startSequence(SequenceKind.SHOOT_PURP, target);
            }).named("shootNewPurp");
        }
        return new InstantCommand(()->{});
    }
    public Command shootClosestBall() {
        // Determine closest occupied slot and start a SHOOT_CLOSEST sequence which handles the required delays
        int target = -1;
        if (colorArray[2] != Color.EMPTY) target = 2;
        else if (colorArray[1] != Color.EMPTY) target = 1;
        else if (colorArray[0] != Color.EMPTY) target = 0;

        if (target != -1) {
            return new InstantCommand(() -> {
                int t = -1;
                if (colorArray[2] != Color.EMPTY) t = 2;
                else if (colorArray[1] != Color.EMPTY) t = 1;
                else if (colorArray[0] != Color.EMPTY) t = 0;
                if (t != -1) startSequence(SequenceKind.SHOOT_CLOSEST, t);
            }).named("shootClosestBall");
        }
        return new InstantCommand(() -> {}).named("shootClosestBall");
    }

}


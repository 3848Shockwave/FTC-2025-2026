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

/**
 * Sort subsystem
 * ----------------
 * Overview:
 * This class controls a 3-slot spindex (rotating indexer) and a two-servo pusher (scissor) used to
 * eject balls. It was refactored to avoid scheduling commands from within other commands. Instead,
 * higher-level requesters trigger short InstantCommands which call `startSequence(...)` to configure
 * an internal state machine. The state machine is then advanced from `periodic()` via `processSequence()`.
 *
 * Key design changes and rationale:
 *  - No command schedules another command. Scheduling a command from inside another command causes
 *    nested lifecycle complexity and can lead to unexpected concurrency/ordering bugs. To eliminate
 *    that, sequences that previously used SequentialGroup(...) are implemented as an internal state
 *    machine: commands only request a sequence start (via InstantCommand) and `periodic()` executes
 *    the multi-step behavior.
 *  - Timing-sensitive operations (delays, push durations) are implemented with timestamps (ms) and
 *    simple phase variables rather than blocking waits. This works well inside the periodic loop and
 *    preserves responsiveness.
 *
 * State-machine contract (how external commands should interact):
 *  - External commands should NOT directly move the servos or call moveSpindex() while a sequence
 *    is active. Instead, they should call an InstantCommand that calls startSequence(...).
 *  - Commands that drive the intake should be implemented as short-lived LambdaCommands that set
 *    power on start and clear it on stop; they should use `requires(...)` appropriately.
 *  - If you must observe sequence completion, poll `activeSequence == SequenceKind.NONE` or expose
 *    a helper that returns whether the sequence is active. The state machine uses `shootComplete` and
 *    internal fields to track completion state.
 */
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

    private double[] POSITIONS = {0.045, 0.51, 0.999};
    private int currentIndex = 0; // Current slot index (0, 1, or 2)

    /**
     * currentSpindexVelocity
     * - Type: double
     * - Purpose: last measured angular velocity of the spindex (kept for external use if needed)
     * - Units: depends on encoder (rotations/sec or similar)
     * - Note: this field is updated elsewhere via the encoder; used to determine stability in waitToStable()
     */
    private double currentSpindexVelocity = 0.0;

    /**
     * hasRunL / hasRunR
     * - Type: boolean
     * - Purpose: 1-shot guards for cycleLeft and cycleRight LambdaCommands. They prevent repeated
     *   triggering on a single touch press.
     */
    boolean hasRunL = false;
    boolean hasRunR = false;

    /**
     * secondPressSeen
     * - Type: boolean
     * - Purpose: used by pushBallAndBack and sequences to detect the second touch press that signals
     *   the completion of a push/retract action. It is set when touchSensor.isPressed() is observed.
     * - Concurrency: touched by both the LambdaCommand update and the processSequence state machine;
     *   reading it should be considered racy only if accessed outside the periodic loop; prefer exposing
     *   getters if external code must poll it.
     */
    private boolean secondPressSeen = false;


    public enum Color {
        GREEN, PURPLE, EMPTY
    }
    // Assuming index 0 is Right side, index 1 is Left side (Adjust based on physical mounting)
    private Color[] colorArray = {Color.EMPTY, Color.EMPTY, Color.EMPTY};
    private boolean intakeOn = true;

    // === Commands ===
    public Command pushBall = null;

    /**
     * shootComplete
     * - Type: boolean
     * - Purpose: simple flag set when an internal shoot sequence finishes. Mainly for telemetry or tests.
     */
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
    /**
     * SequenceKind
     * - Enum of supported internal sequences. These represent high-level multi-step flows that were
     *   previously implemented as nested command groups. Keeping them internal allows predictable execution
     *   order and avoids the complexity of nested command lifecycles.
     */
    private enum SequenceKind { NONE, SHOOT_GREEN, SHOOT_PURP, SHOOT_CLOSEST }

    /**
     * activeSequence
     * - Type: SequenceKind
     * - Purpose: currently executing sequence. NONE indicates idle.
     * - Concurrency: only modified by startSequence() and processSequence() (both called from the main
     *   thread/periodic loop or InstantCommand invocation). External code should call startSequence via
     *   InstantCommands and may query activeSequence for status.
     */
    private SequenceKind activeSequence = SequenceKind.NONE;

    /**
     * seqTargetIndex
     * - Type: int
     * - Purpose: the logical slot index (0..2) that the active sequence will act upon.
     * - Valid values: -1 (none) or 0..2
     */
    private int seqTargetIndex = -1; // index of the ball we want to handle (0..2)

    /**
     * seqPhase
     * - Type: int
     * - Purpose: small-phase integer used to represent the sub-state within a SequenceKind. We use
     *   numeric phases (e.g., 10/11 for top push, 20/21/22 for left rotate+push) to keep the code
     *   compact and deterministic. Each phase has clear transitions and associated timeouts.
     */
    private int seqPhase = 0;

    /**
     * seqPhaseStartMs
     * - Type: long
     * - Purpose: timestamp (System.currentTimeMillis()) when the current seqPhase started. Used to
     *   implement non-blocking delays.
     */
    private long seqPhaseStartMs = 0;

    // push internal state (used both by pushBallAndBack if scheduled standalone or by sequence)
    /**
     * pushRunning
     * - Type: boolean
     * - Purpose: guard for the push mechanics to indicate an ongoing push-retract cycle.
     */
    private boolean pushRunning = false;

    /**
     * pushStage
     * - Type: int
     * - Purpose: small integer indicating the stage of the push cycle:
     *     0 -> idle
     *     1 -> pusher deployed (pushing)
     *     2 -> pusher retracted (waiting for touch to confirm)
     */
    private int pushStage = 0; // 0 none, 1 pushed, 2 retracted

    /**
     * pushStartMs
     * - Type: long
     * - Purpose: timestamp when the pusher was deployed; used to enforce the deploy duration (e.g., 250ms)
     */
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
        // COMMAND: pushBallAndBack
        // Purpose: perform a deploy->retract push cycle and finish when a second touch-sensor press confirms
        // completion. This command is self-contained: it will not schedule other commands and it will
        // only operate the pusher servos. It uses pushRunning/pushStage/pushStartMs to track progress.
        // Usage notes:
        //  - Designed to run only when the spindex is stable (spindexIsStable == true)
        //  - It sets secondPressSeen when touch sensor reports a press; callers can query that via
        //    getSecondPressSeen() if needed.
        pushBallAndBack = new LambdaCommand()
                .setStart(() -> {
                    secondPressSeen = false;
                    running = false;
                    pushRunning = false;
                    pushStage = 0;
                    pushStartMs = 0;
                })
                .setUpdate(() -> {
                    // Start the push only when the spindex is stable and we haven't seen a confirmation press.
                    if (spindexIsStable && !secondPressSeen && !pushRunning) {
                        // Deploy push (pusher down) - these positions are mechanically defined elsewhere
                        servoLeft.setPosition(-1.0);
                        servoRight.setPosition(1.0);
                        pushStartMs = System.currentTimeMillis();
                        pushRunning = true;
                        pushStage = 1; // now in deployed stage
                    }

                    // After a fixed deploy duration (250ms) retract the pusher. This timing ensures the ball
                    // is fully contacted before retraction starts.
                    if (pushRunning && pushStage == 1) {
                        if (System.currentTimeMillis() - pushStartMs >= 250) {
                            // Retract pusher
                            servoLeft.setPosition(1.0);
                            servoRight.setPosition(-1.0);
                            pushStage = 2; // waiting for touch confirmation
                        }
                    }

                    // If the touch sensor is pressed at any time after the push, mark the second press seen
                    // which signals completion of the push cycle for other logic.
                    if (touchSensor.isPressed()) {
                        secondPressSeen = true;
                    }
                })
                .setIsDone(() -> secondPressSeen)
                .named("pushBallAndBack");




        // Core Rotation Logic

        // COMMAND: cycleLeft
        // Purpose: single-press driven rotation to the left (logical +1). This command is intentionally
        // simple: it listens for a touch press and calls moveSpindex(1) once, then completes.
        // Important: it uses hasRunL as a one-shot guard so repeated periodic() updates don't move the
        // spindex multiple times for a single press. The command should be scheduled only when desired.
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

        // COMMAND: cycleRight
        // Purpose: symmetric to cycleLeft but rotates the spindex right (logical -1).
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


        // COMMAND: positiveIntake
        // Purpose: run the intake motor inward while the command is active. The command sets power on
        // start and clears it on stop. It uses requires(intakeOn, this) gating so other logic can disable
        // the intake via stopIntake.
        positiveIntake = new LambdaCommand()
                .setStart(() -> intake.setPower(1.0))
                .setInterruptible(true)
                .setStop(interrupted -> intake.setPower(0.0))
                .requires(intakeOn, this);

        // COMMAND: negativeIntake
        // Purpose: run the intake motor outward (reverse) while the command is active.
        negativeIntake = new LambdaCommand()
                .setStart(() -> intake.setPower(-1.0))
                .setInterruptible(true)
                .setStop(interrupted -> intake.setPower(0.0))
                .requires(intakeOn, this);


        // Instead of creating SequentialGroup(...) and scheduling it here (which would call commands from within
        // a command), we start an internal sequence handled by periodic()/processSequence(). The InstantCommand
        // below merely requests the sequence start.
        // COMMAND: shootGreen (InstantCommand)
        // Purpose: select the nearest green ball (prefers top -> middle -> bottom) and request the
        // SHOOT_GREEN internal sequence. It does not execute the sequence itself; it only populates
        // the sequence state via startSequence(...) so processSequence() can execute it safely.
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

        // COMMAND: shootPurp (InstantCommand) - symmetric to shootGreen for purple balls
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
        // The state machine is the single-authority executor for sequences. It consumes `activeSequence`,
        // `seqPhase`, `seqTargetIndex` and uses timestamps (`seqPhaseStartMs`, `pushStartMs`) to
        // perform time-based transitions. All actions that change hardware (servo positions, moveSpindex)
        // happen here so that sequences cannot be concurrently partially-executed by other code.

        // High-level design notes for the phases used below:
        //  - Each named sequence (e.g. SHOOT_GREEN) uses seqPhase to represent compact sub-states.
        //  - Numeric ranges are grouped by purpose to make it easy to identify behavior in logs:
        //      10..19  : immediate-top push flow (target index 2)
        //      20..29  : rotate-left then push flow (target index 1)
        //      30..39  : rotate-right then push flow (target index 0)
        //      100..199: SHOOT_CLOSEST top flow
        //      200..299: SHOOT_CLOSEST rotate-left flow
        //      300..399: SHOOT_CLOSEST rotate-right flow
        //  - Transitions are driven by time (seqPhaseStartMs), encoder stability (spindexIsStable),
        //    and touch sensor events. Each transition contains guards to avoid unsafe hardware writes.

        if (activeSequence == SequenceKind.NONE) return;
        long now = System.currentTimeMillis();

        switch (activeSequence) {
            case SHOOT_GREEN:
            case SHOOT_PURP: {
                // These two sequences are identical in physical actions; they differ only in which
                // color triggered them. The state machine therefore handles both in the same branch.

                if (seqPhase == 0) {
                    // Entry: decide which sub-flow to take based on seqTargetIndex.
                    if (seqTargetIndex == 2) {
                        // Target already at top: wait a short delay, then push.
                        seqPhaseStartMs = now;
                        seqPhase = 10; // top flow: initial wait
                    } else if (seqTargetIndex == 1) {
                        // Target in middle: wait for physical confirmation via touch press that a
                        // rotation can be performed (the original code waited for a touch event before
                        // rotating). We mirror that: wait for a press to call moveSpindex(1).
                        seqPhase = 20; // middle flow: wait-for-press-to-rotate-left
                    } else if (seqTargetIndex == 0) {
                        // Target in bottom: symmetric to middle but rotate right (-1)
                        seqPhase = 30; // bottom flow: wait-for-press-to-rotate-right
                    }
                }

                // ------- Top flow (target index 2) -------
                if (seqPhase == 10) {
                    // Safety Delay: let the mechanism settle (1 second) before pushing. This mimics
                    // the prior Delay(1) behavior but does not block; it uses timestamps instead.
                    if (now - seqPhaseStartMs >= 1000) {
                        // Ensure spindex is stable before mechanically interacting with the pusher.
                        if (spindexIsStable) {
                            // Deploy pusher and start timing for the deploy duration
                            servoLeft.setPosition(-1.0);
                            servoRight.setPosition(1.0);
                            pushStartMs = now;
                            pushRunning = true;
                            pushStage = 1;
                            seqPhase = 11; // deployed, awaiting retract & confirmation
                        }
                    }
                }

                if (seqPhase == 11) {
                    // Retract once deploy time has elapsed, then wait for touch confirmation (second press)
                    if (pushRunning && pushStage == 1) {
                        // After the deploy duration (250ms) start retract sequence
                        if (now - pushStartMs >= 250) {
                            servoLeft.setPosition(1.0);
                            servoRight.setPosition(-1.0);
                            pushStage = 2; // retracted, now expect a touch press to confirm
                        }
                    }
                    if (pushStage == 2 && touchSensor.isPressed()) {
                        // Touch confirms the ball left the system. Update memory and finish sequence.
                        colorArray[2] = Color.EMPTY;
                        activeSequence = SequenceKind.NONE;
                        seqPhase = 0;
                        pushRunning = false;
                        pushStage = 0;
                        shootComplete = true;
                    }
                }

                // ------- Middle flow (target index 1) -------
                if (seqPhase == 20) {
                    // Wait for an operator/automation to press the touch sensor indicating the spindex
                    // can rotate left. This preserves original behavior that used cycleLeft which
                    // depended on a press event.
                    if (touchSensor.isPressed()) {
                        moveSpindex(1);
                        // After moving we rely on the encoder stability check before pushing.
                        seqPhase = 21;
                        seqPhaseStartMs = now; // mark when rotation happened
                    }
                }
                if (seqPhase == 21) {
                    // Wait for encoder-stability and a very short buffer (100ms) to allow mechanical
                    // settling before initiating a push. We explicitly require spindexIsStable to avoid
                    // pushing while the indexer is still moving.
                    if (spindexIsStable) {
                        if (now - seqPhaseStartMs >= 100) {
                            // Deploy pusher
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
                    // Same deploy/retract/confirm pattern as top flow
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

                // ------- Bottom flow (target index 0) -------
                if (seqPhase == 30) {
                    // Wait for a press to rotate right (operator-assisted rotation), matching original semantics.
                    if (touchSensor.isPressed()) {
                        moveSpindex(-1);
                        seqPhase = 31;
                        seqPhaseStartMs = now;
                    }
                }
                if (seqPhase == 31) {
                    // After rotation, wait for stability then push with the same timings used above
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
                // SHOOT_CLOSEST chooses the nearest non-empty slot and performs a similar push flow but with
                // different timing semantics (shorter initial delays). The phases are grouped so it's clear
                // which chunk belongs to which physical action.

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
                    // Shorter wait (500ms) for momentum to die before pushing
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
                        // Retract after deploy duration
                        servoLeft.setPosition(1.0); servoRight.setPosition(-1.0); pushStage = 2;
                    }
                    if (pushStage == 2) {
                        // No touch-confirm required for SHOOT_CLOSEST flow; assume success and clear top
                        colorArray[2] = Color.EMPTY;
                        activeSequence = SequenceKind.NONE; seqPhase = 0; pushRunning=false; pushStage=0;
                    }
                }

                if (seqPhase == 200) {
                    // Rotate-left branch for SHOOT_CLOSEST - requires a press to rotate
                    if (touchSensor.isPressed()) {
                        moveSpindex(1); seqPhase = 201; seqPhaseStartMs = now;
                    }
                }
                if (seqPhase == 201) {
                    // After rotation, wait 500ms + stability check then push
                    if (now - seqPhaseStartMs >= 500 && spindexIsStable) {
                        // Deploy
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


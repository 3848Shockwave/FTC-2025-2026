package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Subsystems.Helpers.Drawing;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.RobotConfig;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.RobotStateTracker;
import org.firstinspires.ftc.teamcode.Subsystems.Coordinator;
import org.firstinspires.ftc.teamcode.Subsystems.Sort;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Auto (Configurable & Vision)")
public class Autonomous extends NextFTCOpMode {

    // Configuration flags
    private boolean isRed = true;       // Default to RED
    private boolean isFar = true;       // Default to FAR side
    private boolean isScore = true;     // Default to SCORE mode
    private Follower follower;

    // Enum to track which autonomous mode was selected
    private enum AutoMode {
        BLUE_FAR_SIDE_SCORE,
        BLUE_GOAL_SIDE_MOVE,
        BLUE_FAR_SIDE_MOVE,
        BLUE_GOAL_SIDE_SCORE,
        RED_FAR_SIDE_SCORE,
        RED_GOAL_SIDE_MOVE,
        RED_FAR_SIDE_MOVE,
        RED_GOAL_SIDE_SCORE
    }

    private AutoMode selectedMode;
    private boolean confirmed = false;
    private boolean readyToGo = false;
    MotorEx intake = new MotorEx("intakeMotor").brakeMode();

    // Paths variables
    private Path simplePath;
    private Path scorePreload;
    private Path moveToCheck;
    private PathChain moveToScore,load1,score2,ready2,score3;
    private PathChain moveBLF;
    private PathChain moveRF;
    private PathChain moveToScoreRF, loadRF1, scoreRF1, loadRF2, scoreRF2;
    private PathChain moveToReady0, moveToPick0, moveToScore0,
            moveToReady1, moveToPick1, moveToScore1,
            moveToReady2, moveToPick2, moveToScore2,
            moveToReady3, moveToPick3, moveToScore3, moveToScore4;

    private RobotStateTracker robotStateTracker = new RobotStateTracker();

    Button x_button, y_button, a_button, b_button;

    public Autonomous() {
        addComponents(
                new SubsystemComponent(Sort.INSTANCE),
                new SubsystemComponent(Coordinator.INSTANCE),
                // ==========================================================
                // CRITICAL FIX: Add Turret Subsystem to the scheduler!
                // Without this, the turret's periodic() code NEVER runs.
                // ==========================================================
                new SubsystemComponent(Turret.INSTANCE),

                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        Drawing.init();
        follower = follower();
        RobotConfig.robotStateTracker = robotStateTracker;

        // Display Turret Position for manual calibration check before start
        telemetry.addData("Turret Position", Turret.INSTANCE.getRealTurretPosition());
        telemetry.update();
    }

    @Override
    public void onWaitForStart() {

        boolean lastX = false, lastY = false, lastA = false;
        confirmed = false;
        while (!isStarted() && !isStopRequested()) {
            if(!confirmed) {
                boolean currentX = gamepad1.x;
                boolean currentY = gamepad1.y;
                boolean currentA = gamepad1.a;

                if (currentX && !lastX) isRed = !isRed;
                if (currentY && !lastY) isFar = !isFar;
                if (currentA && !lastA) isScore = !isScore;

                if(gamepad1.b) {
                    confirmed = true;
                    gamepad1.rumble(250);
                }
                lastX = currentX;
                lastY = currentY;
                lastA = currentA;

                updateSelectedMode();
                updateTelemetry();

                if (follower != null) {
                    follower.update();
                }
                telemetry.update();
            }
            if(confirmed && !readyToGo){
                telemetry.addLine("Configuration Confirmed!");
                telemetry.addData("Selected Mode", selectedMode);
                telemetry.update();
                if (selectedMode == null) return;
                if (RobotConfig.autonomousStartEndPoses != null) {
                    follower.setMaxPower(1);
                    follower.setConstants(Constants.followerConstants);
                    follower.setStartingPose(RobotConfig.autonomousStartEndPoses.getStartPose());
                    follower.update();
                }
                switch (selectedMode) {
                    case BLUE_FAR_SIDE_SCORE:
                        buildBlueFarSideScorePaths();
                        readyToGo = true;
                        break;
                    case BLUE_FAR_SIDE_MOVE:
                        buildBlueFarMovingPaths();
                        readyToGo = true;
                        break;
                    case BLUE_GOAL_SIDE_MOVE:
                        buildBlueGoalMovingPaths();
                        readyToGo = true;
                        break;
                    case BLUE_GOAL_SIDE_SCORE:
                        // WRITE THIS
                        readyToGo = true;
                        break;
                    case RED_FAR_SIDE_SCORE:
                        buildRedFarSideScorePaths();
                        readyToGo = true;
                        break;
                    case RED_FAR_SIDE_MOVE:
                        buildRedFarMovingPaths();
                        readyToGo = true;
                        break;
                    case RED_GOAL_SIDE_MOVE:
                        buildRedGoalMovingPaths();
                        readyToGo = true;
                        break;
                    case RED_GOAL_SIDE_SCORE:
                        buildRedGoalSideScorePaths();
                        readyToGo = true;
                        break;
                }
            }
        }
    }

    private void updateSelectedMode() {
        if (!isRed) { // BLUE
            if (isFar) {
                selectedMode = isScore ? AutoMode.BLUE_FAR_SIDE_SCORE : AutoMode.BLUE_FAR_SIDE_MOVE;
                RobotConfig.autonomousStartEndPoses = isScore ?  RobotConfig.AutonomousStartEndPoses.FARSIDEBLUESCORE :  RobotConfig.AutonomousStartEndPoses.FARSIDEBLUEMOVE ;
            } else { // GOAL
                selectedMode = isScore ? AutoMode.BLUE_GOAL_SIDE_SCORE : AutoMode.BLUE_GOAL_SIDE_MOVE;
                RobotConfig.autonomousStartEndPoses = RobotConfig.AutonomousStartEndPoses.GOALSIDEBLUE;
            }
            RobotConfig.alliance = RobotConfig.Alliance.BLUE;
        } else { // RED
            if (isFar) {
                selectedMode = isScore ? AutoMode.RED_FAR_SIDE_SCORE : AutoMode.RED_FAR_SIDE_MOVE;
                RobotConfig.autonomousStartEndPoses = isScore ?  RobotConfig.AutonomousStartEndPoses.FARSIDEREDSCORE :  RobotConfig.AutonomousStartEndPoses.FARSIDEREDMOVE ;
            } else { // GOAL
                selectedMode = isScore ? AutoMode.RED_GOAL_SIDE_SCORE : AutoMode.RED_GOAL_SIDE_MOVE;
                RobotConfig.autonomousStartEndPoses = RobotConfig.AutonomousStartEndPoses.GOALSIDERED;
            }
            RobotConfig.alliance = RobotConfig.Alliance.RED;
        }
    }

    private void updateTelemetry() {
        telemetry.addLine("=== AUTO CONFIGURATION ===");
        telemetry.addData("Alliance (Square)", isRed ? "RED" : "BLUE");
        telemetry.addData("Side     (Triangle)", isFar ? "FAR" : "GOAL");
        telemetry.addData("Action   (X)", isScore ? "SCORE" : "MOVE");
        telemetry.addData("Confirmation   (Circle)", confirmed ? "CONFIRMED" : "NOT CONFIRMED");
        telemetry.addLine("--------------------------");
        telemetry.addData("SELECTED MODE", selectedMode);
        telemetry.addLine("--------------------------");
        telemetry.addData("Turret Position (deg)", String.format("%.1f", Turret.INSTANCE.getRealTurretPosition()));
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        Turret.INSTANCE.initLimelightSystem();

        switch (selectedMode) {
            case BLUE_FAR_SIDE_SCORE:
                runBlueFarSideScoreCommands().schedule();
                break;
            case BLUE_FAR_SIDE_MOVE:
                runBlueFarMovingCommands().schedule();
                break;
            case BLUE_GOAL_SIDE_MOVE:
                runBlueGoalMovingCommands().schedule();
                break;
            case BLUE_GOAL_SIDE_SCORE:
                telemetry.addData("Warning", "Blue Goal Score Path not implemented yet!");
                break;
            case RED_FAR_SIDE_SCORE:
                runRedFarSideScoreCommands().schedule();
                break;
            case RED_FAR_SIDE_MOVE:
                runRedFarMovingCommands().schedule();
                break;
            case RED_GOAL_SIDE_MOVE:
                runRedGoalMovingCommands().schedule();
                break;
            case RED_GOAL_SIDE_SCORE:
                runRedGoalSideScoreCommands().schedule();
                break;
        }
    }

    @Override
    public void onUpdate() {
        follower.update();
    }

    @Override
    public void onStop() {
        // Safe to disable manual on stop (or keep it off)
        Turret.INSTANCE.setManualControl(false);
        RobotConfig.finalMeasuredPose = follower.getPose();
        RobotConfig.finalMeasuredSpindexPosition = Sort.INSTANCE.getServoPosition();
        super.onStop();
    }

    //  RED GOAL SIDE MOVE
    private void buildRedGoalMovingPaths() {
        simplePath = new Path(new BezierLine(
                RobotConfig.AutonomousStartEndPoses.GOALSIDERED.getStartPose(),
                RobotConfig.AutonomousStartEndPoses.GOALSIDERED.getEndPose()));
        simplePath.setLinearHeadingInterpolation(
                RobotConfig.AutonomousStartEndPoses.GOALSIDERED.getStartPose().getHeading(),
                RobotConfig.AutonomousStartEndPoses.GOALSIDERED.getEndPose().getHeading());
    }

    private Command runRedGoalMovingCommands() {
        return new SequentialGroup(new FollowPath(simplePath));
    }

    // BLUE GOAL SIDE MOVE
    private void buildBlueGoalMovingPaths() {
        simplePath = new Path(new BezierLine(
                RobotConfig.AutonomousStartEndPoses.GOALSIDEBLUE.getStartPose(),
                RobotConfig.AutonomousStartEndPoses.GOALSIDEBLUE.getEndPose()));
        simplePath.setLinearHeadingInterpolation(
                RobotConfig.AutonomousStartEndPoses.GOALSIDEBLUE.getStartPose().getHeading(),
                RobotConfig.AutonomousStartEndPoses.GOALSIDEBLUE.getEndPose().getHeading());
    }

    private Command runBlueGoalMovingCommands() {
        return new SequentialGroup(new FollowPath(simplePath));
    }

    //  BLUE FAR SIDE MOVE
    private void buildBlueFarMovingPaths() {
        moveBLF = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(59.204, 8.500),
                                new Pose(61.793, 22.213),
                                new Pose(34.749, 20.953)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .build();
        telemetry.addLine("Paths Built");
        telemetry.update();
    }

    private Command runBlueFarMovingCommands() {
        return new SequentialGroup(
                new InstantCommand(()->{
                    // ENABLE TRACKING (Turn OFF manual control)
                    Turret.INSTANCE.setManualControl(false);
                    // Turret will now aim automatically using Vision/Blind logic
                }),
                Coordinator.INSTANCE.detectTargetColorArray.thenWait(2),
                new FollowPath(moveBLF)
                // Removed manual angle settings at end, tracking will maintain aim
        );
    }

    // RED FAR SIDE MOVE
    private void buildRedFarMovingPaths() {
        moveRF = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(84.000, 8.500),
                                new Pose(84.411, 25.041),
                                new Pose(109.387, 19.634)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .build();
        telemetry.addLine("Paths Built");
        telemetry.update();
    }

    private Command runRedFarMovingCommands() {
        return new SequentialGroup(
                new InstantCommand(()->{
                    // ENABLE TRACKING
                    Turret.INSTANCE.setManualControl(false);
                }),
                Coordinator.INSTANCE.detectTargetColorArray.thenWait(2),
                new FollowPath(moveRF)
        );
    }

    // ==================== RED FAR SIDE (SCORE) ====================
    private void buildRedFarSideScorePaths() {
        moveToScoreRF = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(84.000, 8.500),
                                new Pose(84.508, 83.717)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(225))
                .build();

        loadRF1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(84.508, 83.717),
                                new Pose(94.073, 57.325),
                                new Pose(119.702, 59.351)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0),.3)
                .build();

        scoreRF1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(119.702, 59.351),
                                new Pose(84.194, 83.832)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))
                .build();

        loadRF2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(84.194, 83.832),
                                new Pose(118.131, 83.529)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0),.2)
                .build();

        scoreRF2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(118.131, 83.529),
                                new Pose(84.435, 83.670)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))
                .build();
        telemetry.addLine("Paths Built");
        telemetry.update();
    }

    private Command runRedFarSideScoreCommands() {
        return new SequentialGroup(
                new InstantCommand(()->{
                    // ENABLE TRACKING for the entire sequence
                    Turret.INSTANCE.setManualControl(false);
                }),
                Coordinator.INSTANCE.detectTargetColorArray.thenWait(2),

                // The turret will automatically aim at the RED GOAL while moving
                new FollowPath(moveToScoreRF),

                // No need to set angles manually, logic handles it.
                // Just control Intake/Shooting
                new Delay(3.5), // Wait for shot?

                new FollowPath(loadRF1).and(
                        new InstantCommand(()->{
                            intake.setPower(1.0);
                        })
                ),
                new FollowPath(scoreRF1).and(
                        new InstantCommand(()->{
                            intake.setPower(0);
                        })
                ).thenWait(3.5),

                new FollowPath(loadRF2).and(
                        new InstantCommand(()->{
                            intake.setPower(1.0);
                        })),

                new FollowPath(scoreRF2).and(new InstantCommand(()->{
                    intake.setPower(0.0);
                })).thenWait(5)
        );
    }

    // ==================== BLUE FAR SIDE (SCORE) ====================
    private void buildBlueFarSideScorePaths() {
        moveToScore = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(59.204, 8.5),
                                new Pose(59.251, 84.471)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(315))
                .build();

        load1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(59.251, 84.471),
                                new Pose(54.895, 52.436),
                                new Pose(44.010, 62.387),
                                new Pose(21.107, 59.887)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180),.4)
                .build();

        score2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(21.107, 59.887),
                                new Pose(59.273, 84.435)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                .build();

        ready2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(59.273, 84.435),
                                new Pose(21.836, 83.855)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180),.2)
                .build();

        score3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(21.836, 83.855),
                                new Pose(47.670, 95.450)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                .build();

        telemetry.addLine("Paths Built");
        telemetry.update();
    }

    private Command runBlueFarSideScoreCommands() {
        return new SequentialGroup(
                new InstantCommand(()->{
                    // ENABLE TRACKING
                    Turret.INSTANCE.setManualControl(false);
                }),
                Coordinator.INSTANCE.detectTargetColorArray.thenWait(2),

                new FollowPath(moveToScore),
                new Delay(3.5),

                new FollowPath(load1).and(
                        new InstantCommand(()->{
                            intake.setPower(1.0);
                        })
                ),
                new FollowPath(score2).and(
                        new InstantCommand(()->{
                            intake.setPower(0);
                        })
                ).thenWait(3.5),

                new FollowPath(ready2).and(
                        new InstantCommand(()->{
                            intake.setPower(1.0);
                        })),

                new FollowPath(score3).and(new InstantCommand(()->{
                    intake.setPower(0.0);
                }))
        );
    }

    // ==================== RED GOAL SIDE (SCORE) ====================
    private void buildRedGoalSideScorePaths() {
        Pose startPose = RobotConfig.AutonomousStartEndPoses.GOALSIDERED.getStartPose();
        Pose check = new Pose(125, 122, Math.toRadians(270));
        Pose Score1 = new Pose(85, 96, Math.toRadians(217));
        Pose move1 = new Pose(85, 83, Math.toRadians(0));
        Pose grab1 = new Pose(120, 83, Math.toRadians(0));
        Pose Score2 = new Pose(90, 90, Math.toRadians(217));
        Pose move2 = new Pose(90, 60, Math.toRadians(0));
        Pose grab2 = new Pose(120, 60, Math.toRadians(0));
        Pose Score3 = new Pose(87, 87, Math.toRadians(217));
        Pose move3 = new Pose(87, 35, Math.toRadians(0));
        Pose grab3 = new Pose(120, 35, Math.toRadians(0));
        Pose Score4 = new Pose(80, 80, Math.toRadians(217));

        moveToCheck = new Path(new BezierLine(startPose, check));
        moveToCheck.setLinearHeadingInterpolation(startPose.getHeading(), check.getHeading());

        scorePreload = new Path(new BezierLine(check, Score1));
        scorePreload.setLinearHeadingInterpolation(check.getHeading(), Score1.getHeading());

        moveToReady1 = follower.pathBuilder()
                .addPath(new BezierLine(Score1, move1))
                .setLinearHeadingInterpolation(Score1.getHeading(), move1.getHeading())
                .build();

        moveToPick1 = follower.pathBuilder()
                .addPath(new BezierLine(move1, grab1))
                .setLinearHeadingInterpolation(move1.getHeading(), grab1.getHeading())
                .build();

        moveToScore2 = follower.pathBuilder()
                .addPath(new BezierLine(grab1, Score2))
                .setLinearHeadingInterpolation(grab1.getHeading(), Score2.getHeading())
                .build();

        moveToReady2 = follower.pathBuilder()
                .addPath(new BezierLine(Score2, move2))
                .setLinearHeadingInterpolation(Score2.getHeading(), move2.getHeading())
                .build();

        moveToPick2 = follower.pathBuilder()
                .addPath(new BezierLine(move2, grab2))
                .setLinearHeadingInterpolation(move2.getHeading(), grab2.getHeading())
                .build();

        moveToScore3 = follower.pathBuilder()
                .addPath(new BezierLine(grab2, Score3))
                .setLinearHeadingInterpolation(grab2.getHeading(), Score3.getHeading())
                .build();

        moveToReady3 = follower.pathBuilder()
                .addPath(new BezierLine(Score3, move3))
                .setLinearHeadingInterpolation(Score3.getHeading(), move3.getHeading())
                .build();

        moveToPick3 = follower.pathBuilder()
                .addPath(new BezierLine(move3, grab3))
                .setLinearHeadingInterpolation(move3.getHeading(), grab3.getHeading())
                .build();

        moveToScore4 = follower.pathBuilder()
                .addPath(new BezierLine(grab3, Score4))
                .setLinearHeadingInterpolation(grab3.getHeading(), Score4.getHeading())
                .build();
        telemetry.addLine("Paths Built");
        telemetry.update();
    }

    private Command runRedGoalSideScoreCommands() {
        return new SequentialGroup(
                // Enable Tracking
                new InstantCommand(() -> Turret.INSTANCE.setManualControl(false)),

                new FollowPath(moveToCheck),
                new FollowPath(scorePreload),

                // Cycles
                new FollowPath(moveToReady1),
                new FollowPath(moveToPick1),
                new FollowPath(moveToScore2),

                new FollowPath(moveToReady2),
                new FollowPath(moveToPick2),
                new FollowPath(moveToScore3),

                new FollowPath(moveToReady3),
                new FollowPath(moveToPick3),
                new FollowPath(moveToScore4)
        );
    }
}
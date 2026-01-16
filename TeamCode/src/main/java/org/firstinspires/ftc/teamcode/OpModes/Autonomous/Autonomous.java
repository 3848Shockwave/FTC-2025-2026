package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import static dev.nextftc.bindings.Bindings.button;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Subsystems.Helpers.RobotConfig;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.RobotStateTracker;
import org.firstinspires.ftc.teamcode.Subsystems.MySubsystemGroup; // [新增] 导入
import org.firstinspires.ftc.teamcode.Subsystems.Sort;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Auto (Configurable & Vision)")
public class Autonomous extends NextFTCOpMode {

    // Configuration flags
    private boolean isRed = true;       // Default to RED
    private boolean isFar = true;       // Default to FAR side
    private boolean isScore = true;     // Default to SCORE mode

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
    private Follower follower;

    // Paths variables
    private Path simplePath;
    private Path scorePreload;
    private Path moveToCheck;
    private PathChain moveToReady0, moveToPick0, moveBackToScore0,
            moveToReady1, moveToPick1, moveBackToScore1,
            moveToReady2, moveToPick2, moveToScore2,
            moveToReady3, moveToPick3, moveToScore3, moveToScore4;

    private RobotStateTracker robotStateTracker = new RobotStateTracker();

    Button x_button, y_button, a_button, b_button;

    public Autonomous() {
        addComponents(
                new SubsystemComponent(Sort.INSTANCE),
                new SubsystemComponent(MySubsystemGroup.INSTANCE),
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void onWaitForStart() {

        boolean lastX = false, lastY = false, lastA = false;

        while (!isStarted() && !isStopRequested()) {
            boolean currentX = gamepad1.x;
            boolean currentY = gamepad1.y;
            boolean currentA = gamepad1.a;

            if (currentX && !lastX) isRed = !isRed;

            if (currentY && !lastY) isFar = !isFar;

            if (currentA && !lastA) isScore = !isScore;

            lastX = currentX; lastY = currentY; lastA = currentA;

            updateSelectedMode();
            updateTelemetry();

            if(follower != null) follower.update();
            telemetry.update();
        }
    }

    private void updateSelectedMode() {
        if (!isRed) { // BLUE
            if (isFar) {
                selectedMode = isScore ? AutoMode.BLUE_FAR_SIDE_SCORE : AutoMode.BLUE_FAR_SIDE_MOVE;
                RobotConfig.autonomousStartEndPoses = RobotConfig.AutonomousStartEndPoses.FARSIDEBLUE;
            } else { // GOAL
                selectedMode = isScore ? AutoMode.BLUE_GOAL_SIDE_SCORE : AutoMode.BLUE_GOAL_SIDE_MOVE;
                RobotConfig.autonomousStartEndPoses = RobotConfig.AutonomousStartEndPoses.GOALSIDEBLUE;
            }
            RobotConfig.alliance = RobotConfig.Alliance.BLUE;
        } else { // RED
            if (isFar) {
                selectedMode = isScore ? AutoMode.RED_FAR_SIDE_SCORE : AutoMode.RED_FAR_SIDE_MOVE;
                RobotConfig.autonomousStartEndPoses = RobotConfig.AutonomousStartEndPoses.FARSIDERED;
            } else { // GOAL
                selectedMode = isScore ? AutoMode.RED_GOAL_SIDE_SCORE : AutoMode.RED_GOAL_SIDE_MOVE;
                RobotConfig.autonomousStartEndPoses = RobotConfig.AutonomousStartEndPoses.GOALSIDERED;
            }
            RobotConfig.alliance = RobotConfig.Alliance.RED;
        }
    }

    private void updateTelemetry() {
        telemetry.addLine("=== AUTO CONFIGURATION ===");
        telemetry.addData("Alliance (X)", isRed ? "RED" : "BLUE");
        telemetry.addData("Side     (Y)", isFar ? "FAR" : "GOAL");
        telemetry.addData("Action   (A)", isScore ? "SCORE" : "MOVE");
        telemetry.addLine("--------------------------");
        telemetry.addData("SELECTED MODE", selectedMode);
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        if (selectedMode == null) return;

        switch (selectedMode) {
            case BLUE_FAR_SIDE_SCORE:
                buildBlueFarSideScorePaths();
                runBlueFarSideScoreCommands().schedule();
                break;
            case BLUE_FAR_SIDE_MOVE:
                buildBlueFarMovingPaths();
                runBlueFarMovingCommands().schedule();
                break;
            case BLUE_GOAL_SIDE_MOVE:
                buildBlueGoalMovingPaths();
                runBlueGoalMovingCommands().schedule();
                break;
            case BLUE_GOAL_SIDE_SCORE:
                // TODO: add blue goal side score path
                telemetry.addData("Warning", "Blue Goal Score Path not implemented yet!");
                break;
            case RED_FAR_SIDE_SCORE:
                buildRedFarSideScorePaths();
                runRedFarSideScoreCommands().schedule();
                break;
            case RED_FAR_SIDE_MOVE:
                buildRedFarMovingPaths();
                runRedFarMovingCommands().schedule();
                break;
            case RED_GOAL_SIDE_MOVE:
                buildRedGoalMovingPaths();
                runRedGoalMovingCommands().schedule();
                break;
            case RED_GOAL_SIDE_SCORE:
                buildRedGoalSideScorePaths();
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
        RobotConfig.finalMeasuredPose = follower.getPose();
        RobotConfig.finalMeasuredSpindexPosition = Sort.INSTANCE.getServoPosition();
        RobotConfig.robotStateTracker = robotStateTracker;
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
        Pose endpose = new Pose(70, 50, Math.toRadians(310));
        simplePath = new Path(new BezierLine(
                RobotConfig.AutonomousStartEndPoses.FARSIDEBLUE.getStartPose(),
                endpose));
        simplePath.setLinearHeadingInterpolation(
                RobotConfig.AutonomousStartEndPoses.FARSIDEBLUE.getStartPose().getHeading(), // Fixed: was using Red start in original code, but keeping user code as requested if needed. Wait, in prompt it was FARSIDEBLUE. keeping prompt version.
                endpose.getHeading());
    }
    private Command runBlueFarMovingCommands() {
        return new SequentialGroup(new FollowPath(simplePath));
    }

    // RED FAR SIDE MOVE
    private void buildRedFarMovingPaths() {
        Pose endpose = new Pose(80, 50, Math.toRadians(230));
        simplePath = new Path(new BezierLine(
                RobotConfig.AutonomousStartEndPoses.FARSIDERED.getStartPose(),
                endpose));
        simplePath.setLinearHeadingInterpolation(
                RobotConfig.AutonomousStartEndPoses.FARSIDERED.getStartPose().getHeading(),
                endpose.getHeading());
    }
    private Command runRedFarMovingCommands() {
        return new SequentialGroup(new FollowPath(simplePath));
    }



    // ==================== RED FAR SIDE (SCORE) ====================
    private void buildRedFarSideScorePaths() {
        Pose startPose = RobotConfig.AutonomousStartEndPoses.FARSIDERED.getStartPose();
        Pose score = new Pose(96, 86, Math.toRadians(270));
        Pose readyPose0 = new Pose(89, 93, Math.toRadians(0));
        Pose pickUpBalls0 = new Pose(116, 95, Math.toRadians(0));
        Pose readyPose1 = new Pose(89, 71, Math.toRadians(0));
        Pose pickUpBalls1 = new Pose(116, 71, Math.toRadians(0));

        scorePreload = new Path(new BezierLine(startPose, score));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), score.getHeading());

        moveToReady0 = follower.pathBuilder()
                .addPath(new BezierLine(score, readyPose0))
                .setLinearHeadingInterpolation(score.getHeading(), readyPose0.getHeading())
                .build();
        moveToPick0 = follower.pathBuilder()
                .addPath(new BezierLine(readyPose0, pickUpBalls0))
                .setLinearHeadingInterpolation(readyPose0.getHeading(), pickUpBalls0.getHeading())
                .setVelocityConstraint(5)
                .build();
        moveBackToScore0 = follower.pathBuilder()
                .addPath(new BezierLine(pickUpBalls0, score))
                .setLinearHeadingInterpolation(pickUpBalls0.getHeading(), score.getHeading())
                .build();

        moveToReady1 = follower.pathBuilder()
                .addPath(new BezierLine(score, readyPose1))
                .setLinearHeadingInterpolation(score.getHeading(), readyPose1.getHeading())
                .build();
        moveToPick1 = follower.pathBuilder()
                .addPath(new BezierLine(readyPose1, pickUpBalls1))
                .setLinearHeadingInterpolation(readyPose1.getHeading(), pickUpBalls1.getHeading())
                .setVelocityConstraint(5)
                .build();
        moveBackToScore1 = follower.pathBuilder()
                .addPath(new BezierLine(pickUpBalls1, score))
                .setLinearHeadingInterpolation(pickUpBalls1.getHeading(), score.getHeading())
                .build();
    }

    private Command runRedFarSideScoreCommands() {
        return new SequentialGroup(
                // 1. Detect Colors First
                MySubsystemGroup.INSTANCE.detectTargetColorArray,

                // 2. Score Preload (using pattern)
                MySubsystemGroup.INSTANCE.shootInPattern,
                new Delay(0.2), // Optional delay for stabilization

                // 3. Cycle 0
                new FollowPath(moveToReady0),
                new Delay(0.3),
                new FollowPath(moveToPick0).and(Sort.INSTANCE.positiveIntake),
                new Delay(0.3),
                new FollowPath(moveBackToScore0).and(Sort.INSTANCE.positiveIntake).thenWait(0.6),

                // Shoot Cycle 0
                MySubsystemGroup.INSTANCE.shootInPattern,
                new Delay(0.2),

                // 4. Cycle 1
                new FollowPath(moveToReady1),
                new Delay(0.3),
                new FollowPath(moveToPick1).and(Sort.INSTANCE.positiveIntake),
                new Delay(0.35),
                new FollowPath(moveBackToScore1).and(Sort.INSTANCE.positiveIntake).afterTime(0.2).then(Sort.INSTANCE.negativeIntake),

                // Shoot Cycle 1
                MySubsystemGroup.INSTANCE.shootInPattern
        );
    }

    // ==================== BLUE FAR SIDE (SCORE) ====================
    private void buildBlueFarSideScorePaths() {
        Pose startPose = RobotConfig.AutonomousStartEndPoses.FARSIDEBLUE.getStartPose();
        Pose score1 = new Pose(60, 15, Math.toRadians(300));
        Pose readyPose1 = new Pose(55, 35, Math.toRadians(180));
        Pose pickUpBalls1 = new Pose(25, 35, Math.toRadians(180));
        Pose score2 = new Pose(65, 17, Math.toRadians(300));
        Pose ReadyPose2 = new Pose(65, 60, Math.toRadians(180));
        Pose pickUpBalls2 = new Pose(25, 60, Math.toRadians(180));
        Pose score3 = new Pose(50, 84, Math.toRadians(320));
        Pose ReadyPose3 = new Pose(50, 84, Math.toRadians(180));
        Pose pickUpBalls3 = new Pose(25, 84, Math.toRadians(180));
        Pose score4 = new Pose(55, 105, Math.toRadians(320));

        scorePreload = new Path(new BezierLine(startPose, score1));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), score1.getHeading());

        moveToReady1 = follower.pathBuilder()
                .addPath(new BezierLine(score1, readyPose1))
                .setLinearHeadingInterpolation(score1.getHeading(), readyPose1.getHeading())
                .build();

        moveToPick1 = follower.pathBuilder()
                .addPath(new BezierLine(readyPose1, pickUpBalls1))
                .setLinearHeadingInterpolation(readyPose1.getHeading(), pickUpBalls1.getHeading())
                .build();
        moveBackToScore1 = follower.pathBuilder()
                .addPath(new BezierLine(pickUpBalls1, score1))
                .setLinearHeadingInterpolation(pickUpBalls1.getHeading(), score1.getHeading())
                .build();
        moveToReady2 = follower.pathBuilder()
                .addPath(new BezierLine(score1, ReadyPose2))
                .setLinearHeadingInterpolation(score1.getHeading(), ReadyPose2.getHeading())
                .build();
        moveToPick2 = follower.pathBuilder()
                .addPath(new BezierLine(ReadyPose2, pickUpBalls2))
                .setLinearHeadingInterpolation(ReadyPose2.getHeading(), pickUpBalls2.getHeading())
                .build();
        moveToScore3 = follower.pathBuilder()
                .addPath(new BezierLine(pickUpBalls2, score3))
                .setLinearHeadingInterpolation(pickUpBalls2.getHeading(), score3.getHeading())
                .build();
        moveToReady3 = follower.pathBuilder()
                .addPath(new BezierLine(score3, ReadyPose3))
                .setLinearHeadingInterpolation(score3.getHeading(), ReadyPose3.getHeading())
                .build();
        moveToPick3 = follower.pathBuilder()
                .addPath(new BezierLine(ReadyPose3, pickUpBalls3))
                .setLinearHeadingInterpolation(ReadyPose3.getHeading(), pickUpBalls3.getHeading())
                .build();
        moveToScore4 = follower.pathBuilder()
                .addPath(new BezierLine(pickUpBalls3, score4))
                .setLinearHeadingInterpolation(pickUpBalls3.getHeading(), score4.getHeading())
                .build();
    }

    private Command runBlueFarSideScoreCommands() {
        return new SequentialGroup(
                MySubsystemGroup.INSTANCE.detectTargetColorArray,

                new FollowPath(scorePreload),
                MySubsystemGroup.INSTANCE.shootInPattern,

                new FollowPath(moveToReady1).and (Sort.INSTANCE.positiveIntake),
                new FollowPath(moveToPick1).and(Sort.INSTANCE.positiveIntake),
                new FollowPath(moveBackToScore1),
                MySubsystemGroup.INSTANCE.shootInPattern,

                new FollowPath(moveToReady2).and(Sort.INSTANCE.positiveIntake),
                new FollowPath(moveToPick2).and(Sort.INSTANCE.positiveIntake),
                new FollowPath(moveToScore3),
                MySubsystemGroup.INSTANCE.shootInPattern,

                new FollowPath(moveToReady3).and(Sort.INSTANCE.positiveIntake),
                new FollowPath(moveToPick3).and(Sort.INSTANCE.positiveIntake),
                new FollowPath(moveToScore4),
                MySubsystemGroup.INSTANCE.shootInPattern
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
    }

    private Command runRedGoalSideScoreCommands() {
        return new SequentialGroup(

                new FollowPath(moveToCheck),
                // 1. Detect Colors First
                MySubsystemGroup.INSTANCE.detectTargetColorArray,

                // 2. Preload + Shoot
                new FollowPath(scorePreload),
                MySubsystemGroup.INSTANCE.shootInPattern,

                // 3. Cycles
                new FollowPath(moveToReady1),
                new FollowPath(moveToPick1).and(Sort.INSTANCE.positiveIntake),
                new FollowPath(moveToScore2).and(Sort.INSTANCE.positiveIntake).thenWait(0.5),
                MySubsystemGroup.INSTANCE.shootInPattern,

                new FollowPath(moveToReady2),
                new FollowPath(moveToPick2).and(Sort.INSTANCE.positiveIntake),
                new FollowPath(moveToScore3).and(Sort.INSTANCE.positiveIntake).thenWait(0.5),
                MySubsystemGroup.INSTANCE.shootInPattern,

                new FollowPath(moveToReady3),
                new FollowPath(moveToPick3).and(Sort.INSTANCE.positiveIntake),
                new FollowPath(moveToScore4).and(Sort.INSTANCE.positiveIntake).thenWait(0.5),
                MySubsystemGroup.INSTANCE.shootInPattern
        );
    }
}
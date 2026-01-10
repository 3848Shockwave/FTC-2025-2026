package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import static dev.nextftc.bindings.Bindings.button;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Subsystems.Helpers.RobotConfig;
import org.firstinspires.ftc.teamcode.Subsystems.Sort;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
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

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Auto (TESTING)")
public class Autonomous extends NextFTCOpMode {

    // Enum to track which autonomous mode was selected
    private enum AutoMode {
        NONE,
        BLUE_FAR_SIDE,
        BLUE_GOAL_SIDE,
        RED_FAR_SIDE,
        RED_GOAL_SIDE
    }

    private AutoMode selectedMode = AutoMode.NONE;
    private Follower follower;

    // Paths for different autonomous modes
    private Path simplePath;
    private Path scorePreload;
    private PathChain moveToReady0, moveToPick0, moveBackToScore0;
    private PathChain moveToReady1, moveToPick1, moveBackToScore1;
    private PathChain moveToReady2, moveToPick2, moveBackToScore2;

    Button x_button, y_button, a_button, b_button;

    public Autonomous() {
        addComponents(
                new SubsystemComponent(Sort.INSTANCE),
                //new SubsystemComponent(Turret.INSTANCE),
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {

        telemetry.addData("Select an Auto",
                "\nPress X for BLUE FAR SIDE autonomous" +
                        "\nPress Y for BLUE GOAL SIDE autonomous" +
                        "\nPress A for RED FAR SIDE autonomous" +
                        "\nPress B for RED GOAL SIDE autonomous");
        telemetry.update();

        x_button = button(() -> gamepad1.x);
        y_button = button(() -> gamepad1.y);
        a_button = button(() -> gamepad1.a);
        b_button = button(() -> gamepad1.b);

        x_button.whenBecomesTrue(() -> {
            selectedMode = AutoMode.BLUE_FAR_SIDE;
            RobotConfig.alliance = RobotConfig.Alliance.BLUE;
            RobotConfig.autonomousStartEndPoses = RobotConfig.AutonomousStartEndPoses.FARSIDEBLUE;
            telemetry.addData("Auto Selected:", "BLUE FAR SIDE autonomous");
            telemetry.update();
        });

        y_button.whenBecomesTrue(() -> {
            selectedMode = AutoMode.BLUE_GOAL_SIDE;
            RobotConfig.alliance = RobotConfig.Alliance.BLUE;
            RobotConfig.autonomousStartEndPoses = RobotConfig.AutonomousStartEndPoses.GOALSIDEBLUE;
            telemetry.addData("Auto Selected:", "BLUE GOAL SIDE autonomous");
            telemetry.update();
        });

        a_button.whenBecomesTrue(() -> {
            selectedMode = AutoMode.RED_FAR_SIDE;
            RobotConfig.alliance = RobotConfig.Alliance.RED;
            RobotConfig.autonomousStartEndPoses = RobotConfig.AutonomousStartEndPoses.FARSIDERED;
            telemetry.addData("Auto Selected:", "RED FAR SIDE autonomous");
            telemetry.update();
        });

        b_button.whenBecomesTrue(() -> {
            selectedMode = AutoMode.RED_GOAL_SIDE;
            RobotConfig.alliance = RobotConfig.Alliance.RED;
            RobotConfig.autonomousStartEndPoses = RobotConfig.AutonomousStartEndPoses.GOALSIDERED;
            telemetry.addData("Auto Selected:", "RED GOAL SIDE autonomous");
            telemetry.update();
        });
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void onStartButtonPressed() {
        if (selectedMode == AutoMode.NONE) {
            telemetry.addData("ERROR", "No autonomous mode selected!");
            telemetry.update();
            return;
        }

        // Build paths based on selected mode
        switch (selectedMode) {
            case BLUE_FAR_SIDE:
                buildBlueScorePaths();
                runBlueScoreCommands().schedule();
                break;
            case BLUE_GOAL_SIDE:
                buildBlueGoalPaths();
                runBlueGoalCommands().schedule();
                break;
            case RED_FAR_SIDE:
                buildRedScorePaths();
                runRedScoreCommands().schedule();
                break;
            case RED_GOAL_SIDE:
                buildRedGoalPaths();
                runRedGoalCommands().schedule();
                break;
        }

        telemetry.addData("Auto Started", selectedMode.toString());
        telemetry.update();
    }

    @Override
    public void onUpdate() {
        follower.update();
    }

    @Override
    public void onStop() {
        RobotConfig.finalMeasuredPose = follower.getPose();
        RobotConfig.finalMeasuredSpindexPosition = Sort.INSTANCE.getServoPosition();
        super.onStop();
    }

    // ==================== RED GOAL SIDE ====================
    private void buildRedGoalPaths() {
        simplePath = new Path(new BezierLine(
                RobotConfig.AutonomousStartEndPoses.GOALSIDERED.getStartPose(),
                RobotConfig.AutonomousStartEndPoses.GOALSIDERED.getEndPose()));
        simplePath.setLinearHeadingInterpolation(
                RobotConfig.AutonomousStartEndPoses.GOALSIDERED.getStartPose().getHeading(),
                RobotConfig.AutonomousStartEndPoses.GOALSIDERED.getEndPose().getHeading());
    }

    private Command runRedGoalCommands() {
        return new SequentialGroup(
                new FollowPath(simplePath)
        );
    }

    // ==================== BLUE GOAL SIDE ====================
    private void buildBlueGoalPaths() {
        simplePath = new Path(new BezierLine(
                RobotConfig.AutonomousStartEndPoses.GOALSIDEBLUE.getStartPose(),
                RobotConfig.AutonomousStartEndPoses.GOALSIDEBLUE.getEndPose()));
        simplePath.setLinearHeadingInterpolation(
                RobotConfig.AutonomousStartEndPoses.GOALSIDEBLUE.getStartPose().getHeading(),
                RobotConfig.AutonomousStartEndPoses.GOALSIDEBLUE.getEndPose().getHeading());
    }

    private Command runBlueGoalCommands() {
        return new SequentialGroup(
                new FollowPath(simplePath)
        );
    }

    // ==================== RED FAR SIDE (SCORE) ====================
    private void buildRedScorePaths() {
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

    private Command runRedScoreCommands() {
        return new SequentialGroup(
                Sort.INSTANCE.pushBallAndBack,
              //  new FollowPath(scorePreload).and(Turret.INSTANCE.RunTurret).thenWait(.5),
                Sort.INSTANCE.pushBallAndBack,
                new Delay(.45),
                Sort.INSTANCE.cycleLeft.endAfter(.9),
                Sort.INSTANCE.pushBallAndBack,
                new Delay(.45),
                Sort.INSTANCE.cycleLeft.endAfter(.9),
                Sort.INSTANCE.pushBallAndBack.thenWait(.2),
                new FollowPath(moveToReady0),
                new Delay(.3),
                new FollowPath(moveToPick0).and(Sort.INSTANCE.positiveIntake),
                new Delay(.3),
                new FollowPath(moveBackToScore0).and(Sort.INSTANCE.positiveIntake).thenWait(.6),
                Sort.INSTANCE.pushBallAndBack,
                new Delay(.45),
                Sort.INSTANCE.cycleLeft.endAfter(.9),
                Sort.INSTANCE.pushBallAndBack,
                new Delay(.45),
                Sort.INSTANCE.cycleLeft.endAfter(.9),
                Sort.INSTANCE.pushBallAndBack.thenWait(.2),
                new FollowPath(moveToReady1),
                new Delay(.3),
                new FollowPath(moveToPick1).and(Sort.INSTANCE.positiveIntake),
                new Delay(.35),
                new FollowPath(moveBackToScore1).and(Sort.INSTANCE.positiveIntake).afterTime(.2).then(Sort.INSTANCE.negativeIntake),
                Sort.INSTANCE.pushBallAndBack,
                new Delay(.45),
                Sort.INSTANCE.cycleLeft.endAfter(.9),
                Sort.INSTANCE.pushBallAndBack,
                new Delay(.45),
                Sort.INSTANCE.cycleLeft.endAfter(.9),
                Sort.INSTANCE.pushBallAndBack.thenWait(.2)
        );
    }

    // ==================== BLUE FAR SIDE (SCORE) ====================
    private void buildBlueScorePaths() {
        Pose startPose = RobotConfig.AutonomousStartEndPoses.FARSIDEBLUE.getStartPose();
        Pose score = new Pose(144 - 70, 86, Math.toRadians(270));
        Pose readyPose0 = new Pose(65, 81, Math.toRadians(180));
        Pose pickUpBalls0 = new Pose(38, 83, Math.toRadians(180));
        Pose readyPose1 = new Pose(65, 59, Math.toRadians(180));
        Pose pickUpBalls1 = new Pose(38, 59, Math.toRadians(180));

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

    private Command runBlueScoreCommands() {
        return new SequentialGroup(
                Sort.INSTANCE.pushBallAndBack,
               // new FollowPath(scorePreload).and(Turret.INSTANCE.RunTurret).thenWait(.5),
                Sort.INSTANCE.pushBallAndBack,
                new Delay(.45),
                Sort.INSTANCE.cycleLeft.endAfter(.9),
                Sort.INSTANCE.pushBallAndBack,
                new Delay(.45),
                Sort.INSTANCE.cycleLeft.endAfter(.9),
                Sort.INSTANCE.pushBallAndBack.thenWait(.2),
                new FollowPath(moveToReady0),
                new Delay(.3),
                new FollowPath(moveToPick0).and(Sort.INSTANCE.positiveIntake),
                new Delay(.3),
                new FollowPath(moveBackToScore0).and(Sort.INSTANCE.positiveIntake).thenWait(.6),
                Sort.INSTANCE.pushBallAndBack,
                new Delay(.45),
                Sort.INSTANCE.cycleLeft.endAfter(.9),
                Sort.INSTANCE.pushBallAndBack,
                new Delay(.45),
                Sort.INSTANCE.cycleLeft.endAfter(.9),
                Sort.INSTANCE.pushBallAndBack.thenWait(.2),
                new FollowPath(moveToReady1),
                new Delay(.3),
                new FollowPath(moveToPick1).and(Sort.INSTANCE.positiveIntake),
                new Delay(.35),
                new FollowPath(moveBackToScore1).and(Sort.INSTANCE.positiveIntake).afterTime(.2).then(Sort.INSTANCE.negativeIntake),
                Sort.INSTANCE.pushBallAndBack,
                new Delay(.45),
                Sort.INSTANCE.cycleLeft.endAfter(.9),
                Sort.INSTANCE.pushBallAndBack,
                new Delay(.45),
                Sort.INSTANCE.cycleLeft.endAfter(.9),
                Sort.INSTANCE.pushBallAndBack.thenWait(.2)
        );
    }
}
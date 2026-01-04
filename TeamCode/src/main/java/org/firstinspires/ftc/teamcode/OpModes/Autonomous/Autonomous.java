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

    Button x_button, y_button, a_button, b_button;


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
            RobotConfig.alliance = RobotConfig.Alliance.BLUE;
            RobotConfig.autonomousStartEndPoses = RobotConfig.AutonomousStartEndPoses.FARSIDEBLUE;
            telemetry.addData("Auto Selected:", " BLUE FAR SIDE autonomous");
            telemetry.update();
            AutonomousBlueScore autoBlueScore = new AutonomousBlueScore();
            autoBlueScore.runOpMode();
        });
        y_button.whenBecomesTrue(() -> {
            RobotConfig.alliance = RobotConfig.Alliance.BLUE;
            RobotConfig.autonomousStartEndPoses = RobotConfig.AutonomousStartEndPoses.GOALSIDEBLUE;
            telemetry.addData("Auto Selected:", " BLUE GOAL SIDE autonomous");
            telemetry.update();
            AutonomousBlueGoal autoBlueGoal = new AutonomousBlueGoal();
            autoBlueGoal.runOpMode();
        });
        a_button.whenBecomesTrue(() -> {
            RobotConfig.alliance = RobotConfig.Alliance.RED;
            RobotConfig.autonomousStartEndPoses = RobotConfig.AutonomousStartEndPoses.FARSIDERED;
            telemetry.addData("Auto Selected:", " RED FAR SIDE autonomous");
            telemetry.update();
            AutonomousRedScore autoRedScore = new AutonomousRedScore();
            autoRedScore.runOpMode();
        });
        b_button.whenBecomesTrue(() -> {
            RobotConfig.alliance = RobotConfig.Alliance.RED;
            RobotConfig.autonomousStartEndPoses = RobotConfig.AutonomousStartEndPoses.GOALSIDERED;
            telemetry.addData("Auto Selected:", " RED GOAL SIDE autonomous");
            telemetry.update();
            AutonomousRedGoal autoRedGoal = new AutonomousRedGoal();
            autoRedGoal.runOpMode();
        });
    }
}

class AutonomousRedGoal extends NextFTCOpMode {

    private Follower follower;
    private Path move;

    //add components within this constructor
    public AutonomousRedGoal() {
        addComponents(
                new SubsystemComponent(Sort.INSTANCE),
                new SubsystemComponent(Turret.INSTANCE),
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    private Command runAutoCommands() {
        return new SequentialGroup(
                new FollowPath(move)
        );

    }

    public void onStop() {
        RobotConfig.finalMeasuredPose = follower.getPose();
        super.onStop();
    }

    public void buildPaths() {
        move = new Path(new BezierLine(RobotConfig.AutonomousStartEndPoses.GOALSIDERED.getStartPose(), RobotConfig.AutonomousStartEndPoses.GOALSIDERED.getEndPose()));
        move.setLinearHeadingInterpolation(RobotConfig.AutonomousStartEndPoses.GOALSIDERED.getStartPose().getHeading(), RobotConfig.AutonomousStartEndPoses.GOALSIDERED.getEndPose().getHeading());

    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void onUpdate() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();

    }

    @Override
    public void onStartButtonPressed() {
        buildPaths();
        runAutoCommands().schedule();
        telemetry.addData("Auto Started", "");
        telemetry.update();
    }
}


class AutonomousBlueGoal extends NextFTCOpMode {

    private Follower follower;
    private Path move;


    //add components within this constructor
    public AutonomousBlueGoal() {
        addComponents(
                new SubsystemComponent(Sort.INSTANCE),
                new SubsystemComponent(Turret.INSTANCE),
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    public void onStop() {
        RobotConfig.finalMeasuredPose = follower.getPose();
        super.onStop();
    }

    private Command runAutoCommands() {
        return new SequentialGroup(
                new FollowPath(move)
        );

    }

    public void buildPaths() {
        move = new Path(new BezierLine(RobotConfig.AutonomousStartEndPoses.GOALSIDEBLUE.getStartPose(), RobotConfig.AutonomousStartEndPoses.GOALSIDEBLUE.getEndPose()));
        move.setLinearHeadingInterpolation(RobotConfig.AutonomousStartEndPoses.GOALSIDEBLUE.getStartPose().getHeading(), RobotConfig.AutonomousStartEndPoses.GOALSIDEBLUE.getEndPose().getHeading());

    }


    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void onUpdate() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();

    }


    @Override
    public void onStartButtonPressed() {
        buildPaths();
        runAutoCommands().schedule();
        telemetry.addData("Auto Started", "");
        telemetry.update();
    }
}

class AutonomousRedScore extends NextFTCOpMode {

    private Follower follower;
    private final Pose startPose = RobotConfig.AutonomousStartEndPoses.FARSIDERED.getStartPose();// Start Pose of our robot.
    private final Pose score = new Pose(96, 86, Math.toRadians(270)); // Scoring Pose of our robot.
    private final Pose readyPose0 = new Pose(89, 93, Math.toRadians(0));
    private final Pose pickUpBalls0 = new Pose(116, 95, Math.toRadians(0));
    private final Pose readyPose1 = new Pose(89, 71, Math.toRadians(0));
    private final Pose pickUpBalls1 = new Pose(116, 71, Math.toRadians(0));
    private final Pose readyPose2 = new Pose(89, 36, Math.toRadians(0));
    private final Pose pickUpBalls2 = new Pose(116, 36, Math.toRadians(0));
    private Path scorePreload;
    private PathChain moveToReady0, moveToPick0, moveBackToScore0, moveToReady1, moveToPick1, moveBackToScore1,
            moveToReady2, moveToPick2, moveBackToScore2;

    public AutonomousRedScore() {
        addComponents(
                new SubsystemComponent(Sort.INSTANCE),
                new SubsystemComponent(Turret.INSTANCE),
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    public void onStop() {
        RobotConfig.finalMeasuredPose = follower.getPose();
        super.onStop();
    }

    private Command runAutoCommands() {
        return new SequentialGroup(
                Sort.INSTANCE.pushBallAndBack,
                new FollowPath(scorePreload).and(Turret.INSTANCE.RunTurret).thenWait(.5),
                Sort.INSTANCE.pushBallAndBack,
                new Delay(.45),
                Sort.INSTANCE.cycleLeftAuto.endAfter(.9),
                Sort.INSTANCE.pushBallAndBack,
                new Delay(.45),
                Sort.INSTANCE.cycleLeftAuto.endAfter(.9),
                Sort.INSTANCE.pushBallAndBack.thenWait(.2),
                new FollowPath(moveToReady0),
                new Delay(.3),
                new FollowPath(moveToPick0).and(Sort.INSTANCE.positiveIntake),
                new Delay(.3),
                new FollowPath(moveBackToScore0).and(Sort.INSTANCE.positiveIntake).thenWait(.6),
                Sort.INSTANCE.pushBallAndBack,
                new Delay(.45),
                Sort.INSTANCE.cycleLeftAuto.endAfter(.9),
                Sort.INSTANCE.pushBallAndBack,
                new Delay(.45),
                Sort.INSTANCE.cycleLeftAuto.endAfter(.9),
                Sort.INSTANCE.pushBallAndBack.thenWait(.2),
                new FollowPath(moveToReady1),
                new Delay(.3),
                new FollowPath(moveToPick1).and(Sort.INSTANCE.positiveIntake),
                new Delay(.35),
                new FollowPath(moveBackToScore1).and(Sort.INSTANCE.positiveIntake).afterTime(.2).then(Sort.INSTANCE.negativeIntake),
                Sort.INSTANCE.pushBallAndBack,
                new Delay(.45),
                Sort.INSTANCE.cycleLeftAuto.endAfter(.9),
                Sort.INSTANCE.pushBallAndBack,
                new Delay(.45),
                Sort.INSTANCE.cycleLeftAuto.endAfter(.9),
                Sort.INSTANCE.pushBallAndBack.thenWait(.2)

        );


    }

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
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
        moveToReady2 = follower.pathBuilder()
                .addPath(new BezierLine(score, readyPose2))
                .setLinearHeadingInterpolation(score.getHeading(), readyPose2.getHeading())
                .build();
        moveToPick2 = follower.pathBuilder()
                .addPath(new BezierLine(readyPose2, pickUpBalls2))
                .setLinearHeadingInterpolation(readyPose2.getHeading(), pickUpBalls2.getHeading())
                .build();
        moveBackToScore2 = follower.pathBuilder()
                .addPath(new BezierLine(pickUpBalls2, score))
                .setLinearHeadingInterpolation(pickUpBalls2.getHeading(), score.getHeading())
                .build();


    }


    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void onUpdate() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();

    }


    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void onStartButtonPressed() {
        buildPaths();
        runAutoCommands().schedule();
        telemetry.addData("Auto Started", "");
        telemetry.update();
    }
}

class AutonomousBlueScore extends NextFTCOpMode {

    private Follower follower;
    private final Pose startPose = RobotConfig.AutonomousStartEndPoses.FARSIDEBLUE.getStartPose();// Start Pose of our robot.
    private final Pose score = new Pose(144 - 70, 86, Math.toRadians(270));
    private final Pose readyPose0 = new Pose(65, 81, Math.toRadians(180));
    private final Pose pickUpBalls0 = new Pose(38, 83, Math.toRadians(180));
    // Scoring Pose of our robot.
    private final Pose readyPose2 = new Pose(65, 36, Math.toRadians(180));
    private final Pose pickUpBalls2 = new Pose(25, 36, Math.toRadians(180));
    private Path scorePreload;    private Pose readyPose1 = readyPose1 = new Pose(65, 59, Math.toRadians(180));
    private PathChain moveToReady0, moveToPick0, moveBackToScore0, moveToReady1, moveToPick1, moveBackToScore1,
            moveToReady2, moveToPick2, moveBackToScore2;    private Pose pickUpBalls1 = pickUpBalls1 = new Pose(38, 59, Math.toRadians(180));
    public AutonomousBlueScore() {
        addComponents(
                new SubsystemComponent(Sort.INSTANCE),
                new SubsystemComponent(Turret.INSTANCE),
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    @Override
    public void onStop() {
        RobotConfig.finalMeasuredPose = follower.getPose();
        super.onStop();
    }

    private Command runAutoCommands() {
        return new SequentialGroup(
                Sort.INSTANCE.pushBallAndBack,
                new FollowPath(scorePreload).and(Turret.INSTANCE.RunTurret).thenWait(.5),
                Sort.INSTANCE.pushBallAndBack,
                new Delay(.45),
                Sort.INSTANCE.cycleLeftAuto.endAfter(.9),
                Sort.INSTANCE.pushBallAndBack,
                new Delay(.45),
                Sort.INSTANCE.cycleLeftAuto.endAfter(.9),
                Sort.INSTANCE.pushBallAndBack.thenWait(.2),
                new FollowPath(moveToReady0),
                new Delay(.3),
                new FollowPath(moveToPick0).and(Sort.INSTANCE.positiveIntake),
                new Delay(.3),
                new FollowPath(moveBackToScore0).and(Sort.INSTANCE.positiveIntake).thenWait(.6),
                Sort.INSTANCE.pushBallAndBack,
                new Delay(.45),
                Sort.INSTANCE.cycleLeftAuto.endAfter(.9),
                Sort.INSTANCE.pushBallAndBack,
                new Delay(.45),
                Sort.INSTANCE.cycleLeftAuto.endAfter(.9),
                Sort.INSTANCE.pushBallAndBack.thenWait(.2),
                new FollowPath(moveToReady1),
                new Delay(.3),
                new FollowPath(moveToPick1).and(Sort.INSTANCE.positiveIntake),
                new Delay(.35),
                new FollowPath(moveBackToScore1).and(Sort.INSTANCE.positiveIntake).afterTime(.2).then(Sort.INSTANCE.negativeIntake),
                Sort.INSTANCE.pushBallAndBack,
                new Delay(.45),
                Sort.INSTANCE.cycleLeftAuto.endAfter(.9),
                Sort.INSTANCE.pushBallAndBack,
                new Delay(.45),
                Sort.INSTANCE.cycleLeftAuto.endAfter(.9),
                Sort.INSTANCE.pushBallAndBack.thenWait(.2)

        );


    }

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
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
        moveToReady2 = follower.pathBuilder()
                .addPath(new BezierLine(score, readyPose2))
                .setLinearHeadingInterpolation(score.getHeading(), readyPose2.getHeading())
                .build();
        moveToPick2 = follower.pathBuilder()
                .addPath(new BezierLine(readyPose2, pickUpBalls2))
                .setLinearHeadingInterpolation(readyPose2.getHeading(), pickUpBalls2.getHeading())
                .build();
        moveBackToScore2 = follower.pathBuilder()
                .addPath(new BezierLine(pickUpBalls2, score))
                .setLinearHeadingInterpolation(pickUpBalls2.getHeading(), score.getHeading())
                .build();

    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void onUpdate() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();

    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void onStartButtonPressed() {
        buildPaths();
        runAutoCommands().schedule();
        telemetry.addData("Auto Started", "");
        telemetry.update();
    }







}





















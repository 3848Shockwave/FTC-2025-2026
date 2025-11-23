package org.firstinspires.ftc.teamcode.OpModes.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.Sort;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "Example Auto")
public class PurePP extends NextFTCOpMode {

    //add components within this constructor
    public PurePP(){
        addComponents(
                new SubsystemComponent(Sort.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(60, 10, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose score = new Pose(70,84, Math.toRadians(270)); // Scoring Pose of our robot.
    private final Pose pickUpBalls0 = new Pose(25, 84, Math.toRadians(180));
    private final Pose readyPose1 = new Pose(60,60, Math.toRadians(180));
    private final Pose pickUpBalls1 = new Pose(25, 60, Math.toRadians(180));
    private final Pose readyPose2 = new Pose (60,36, Math.toRadians(180));
    private final Pose pickUpBalls2 = new Pose(25, 36, Math.toRadians(180));

    private Path scorePreload;
    private PathChain moveToPick0, moveBackToScore0, moveToReady1, moveToPick1, moveBackToScore1,
            moveToReady2, moveToPick2, moveBackToScore2;

    private Command test() {
        return new SequentialGroup(
            Sort.INSTANCE.pushBallAndBack,
                new FollowPath(scorePreload)
//                new Delay(2),
//                new FollowPath(moveToPick0),
//                new Delay(2),
//                new FollowPath(moveBackToScore0),
//                new Delay(2),
//                new FollowPath(moveToReady1),
//                new Delay(2),
//                new FollowPath(moveToPick1),
//                new Delay(2),
//                new FollowPath(moveBackToScore1),
//                new Delay(2),
//                new FollowPath(moveToReady2),
//                new Delay(2),
//                new FollowPath(moveToPick2),
//                new Delay(2),
//                new FollowPath(moveBackToScore2)
        );
    }

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, score));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), score.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */


        moveToPick0 = follower.pathBuilder()
                .addPath(new BezierLine(score, pickUpBalls0))
                .setLinearHeadingInterpolation(score.getHeading(), pickUpBalls0.getHeading())
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
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void onUpdate() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
//        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void onInit() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }


    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        setPathState(0);
        test().schedule();
    }

}
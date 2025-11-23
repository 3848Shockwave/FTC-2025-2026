package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;

import dev.nextftc.core.commands.Command;

import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;


import static dev.nextftc.extensions.pedro.PedroComponent.follower;


//check https://nextftc.dev/guide/opmodes/autonomous
//check https://pedropathing.com/docs/pathing/examples/auto


/*
    1.nin auto there always have 2 purple 1 green, no matter the pattern
    2.load all 3 at once and check the inner 2 balls

 */
@Autonomous(name = "NextFTC Autonomous Program Java")
public class AutonomousProgram extends NextFTCOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Path index0, index1, index2, index3;
    private PathChain test0;
    private final Pose startPose = new Pose(60, 12, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose step2 = new Pose(60, 36, Math.toRadians(180));
    private final Pose step3 = new Pose(12, 36, Math.toRadians(90));
    private final Pose step4 = new Pose(72, 36, Math.toRadians(90));
    private final Pose step5 = new Pose(72, 84, Math.toRadians(90));
    private final Pose scorePose = new Pose(72, 84, Math.toRadians(90));



    public void buildPaths(){

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        index0 = new Path(new BezierLine(startPose, step2));
        index0.setLinearHeadingInterpolation(startPose.getHeading(), step2.getHeading());

        index1 = new Path(new BezierLine(step2, step3));
        index1.setLinearHeadingInterpolation(step2.getHeading(), step3.getHeading());

        index2 = new Path(new BezierLine(step3, step4));
        index2.setLinearHeadingInterpolation(step3.getHeading(), step4.getHeading());

        index3 = new Path(new BezierLine(step4, scorePose));
        index3.setLinearHeadingInterpolation(step4.getHeading(), step5.getHeading());


        /* This is our moveTest1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        test0 = follower().pathBuilder()
                .addPath(new BezierLine(step2, step3))
                .addPath(new BezierLine(step3, step4))
                .setLinearHeadingInterpolation(step2.getHeading(), step3.getHeading())
                .build();

    }

    public AutonomousProgram(){
        addComponents(
                new SubsystemComponent(Drive.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    /*
        sequentialGroup - runs commands one after another
        parallelGroup - runs commands at the same time
     */
    private Command autonomousRoutine(){
        return new SequentialGroup(
                new FollowPath(index0)
//                new FollowPath(index1),
//                new FollowPath(index2),
//                new FollowPath(index3)
        );
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(index0);
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(index1,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(index2,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(index3,true);
                    setPathState(4);
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    @Override
    public void onInit(){
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        setPathState(0);
        // Draw the paths on Panels before starting the routine
//        Tuning.draw();
        autonomousRoutine().schedule();
    }

    @Override
    public void onUpdate(){
        follower.update();
        autonomousPathUpdate();
    }
}
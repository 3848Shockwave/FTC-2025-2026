package org.firstinspires.ftc.teamcode.OpModes;

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
    private Path scorePreload;
    private PathChain moveTest1;


    private final Pose startPose = new Pose(28.5, 128, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(60, 85, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose moveTest1Pose = new Pose(50, 100, Math.toRadians(180));


    public void buildPaths(){

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* This is our moveTest1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        moveTest1 = follower().pathBuilder()
                .addPath(new BezierLine(scorePose, moveTest1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), moveTest1Pose.getHeading())
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
                new FollowPath(scorePreload),
                new FollowPath(moveTest1)
        );
    }

    @Override
    public void onInit(){
        follower().setStartingPose(startPose);
        buildPaths(); // important or the the method FollowPath will reference moveTest1 as null

    }

    @Override
    public void onStartButtonPressed() {
        // Draw the paths on Panels before starting the routine
        Tuning.draw();
        autonomousRoutine().schedule();
    }
}
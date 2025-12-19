package org.firstinspires.ftc.teamcode.OpModes.Autonomous;
import static dev.nextftc.bindings.Bindings.button;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Sort;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnBy;
import dev.nextftc.extensions.pedro.TurnTo;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "Example Auto")
public class PurePP extends NextFTCOpMode {

    //add components within this constructor
    public PurePP(){
        addComponents(
                new SubsystemComponent(Sort.INSTANCE),
                new SubsystemComponent(Turret.INSTANCE),
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private boolean sideSelected = false;
    public Command fireAll=null;

    private  Pose  startPose = new Pose(60, 10, Math.toRadians(90));
    private  Pose score = new Pose(144 - 70, 86, Math.toRadians(270));; // Scoring Pose of our robot.
    private  Pose readyPose0 = new Pose(65, 81, Math.toRadians(180));
    private  Pose pickUpBalls0 =  new Pose(38, 83, Math.toRadians(180));
    private  Pose readyPose1 = readyPose1 = new Pose(65, 59, Math.toRadians(180));
    private  Pose pickUpBalls1 =  pickUpBalls1 = new Pose(38, 59, Math.toRadians(180));
    private  Pose readyPose2 = new Pose(65, 36, Math.toRadians(180));
    private  Pose pickUpBalls2 = new Pose(25, 36, Math.toRadians(180));


    private Path scorePreload;
    private PathChain moveToReady0,moveToPick0, moveBackToScore0, moveToReady1, moveToPick1, moveBackToScore1,
            moveToReady2, moveToPick2, moveBackToScore2;

    private Command test() {
        return new SequentialGroup(
                Sort.INSTANCE.pushBallAndBack,
                    new FollowPath(scorePreload).and(Turret.INSTANCE.RunTurret).thenWait(.5),
                    Sort.INSTANCE.pushBallAndBack.thenWait(.6),
                    new Delay(.45),
                    Sort.INSTANCE.cycleLeftAuto.endAfter(.9),
                    Sort.INSTANCE.pushBallAndBack.thenWait(.6),
                    new Delay(.45),
                    Sort.INSTANCE.cycleLeftAuto.endAfter(.9),
                    Sort.INSTANCE.pushBallAndBack.thenWait(.2),
                    new FollowPath(moveToReady0),
                    new Delay(.3),
                    new FollowPath(moveToPick0).and(Sort.INSTANCE.positiveIntake),
                    new Delay(.3),
                    new FollowPath(moveBackToScore0).and(Sort.INSTANCE.positiveIntake).thenWait(.6),
                    Sort.INSTANCE.pushBallAndBack.thenWait(.6),
                    new Delay(.45),
                    Sort.INSTANCE.cycleLeftAuto.endAfter(.9),
                    Sort.INSTANCE.pushBallAndBack.thenWait(.6),
                    new Delay(.45),
                    Sort.INSTANCE.cycleLeftAuto.endAfter(.9),
                    Sort.INSTANCE.pushBallAndBack.thenWait(.2),
                    new FollowPath(moveToReady1),
                    new Delay(.3),
                    new FollowPath(moveToPick1).and(Sort.INSTANCE.positiveIntake),
                    new Delay(.35),
                    new FollowPath(moveBackToScore1).and(Sort.INSTANCE.positiveIntake).afterTime(.2).then(Sort.INSTANCE.negativeIntake),
                    Sort.INSTANCE.pushBallAndBack.thenWait(.6),
                    new Delay(.45),
                    Sort.INSTANCE.cycleLeftAuto.endAfter(.9),
                    Sort.INSTANCE.pushBallAndBack.thenWait(.6)

        );
    }

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, score));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), score.getHeading());


    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */
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
        if(ActiveOpMode.isStarted()) {
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void onInit() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();




            Button x_button = button(() -> gamepad1.x).whenBecomesTrue(() -> {
                Turret.INSTANCE.setSide("blue");
                sideSelected = true;
                startPose = new Pose(60, 10, Math.toRadians(90)); // Start Pose of our robot.
                score = new Pose(70, 86, Math.toRadians(270)); // Scoring Pose of our robot.
                readyPose0 = new Pose(65, 81, Math.toRadians(180));
                pickUpBalls0 = new Pose(36, 83, Math.toRadians(180));
                readyPose1 = new Pose(65, 59, Math.toRadians(180));
                pickUpBalls1 = new Pose(38, 59, Math.toRadians(180));
                readyPose2 = new Pose(65, 36, Math.toRadians(180));
                pickUpBalls2 = new Pose(25, 36, Math.toRadians(180));
                follower = Constants.createFollower(hardwareMap);
                buildPaths();
                follower.setStartingPose(startPose);
                telemetry.addData("Alliance: ","Blue");
                telemetry.update();

            });

            Button b_button = button(() -> gamepad1.b).whenBecomesTrue(() ->
            {
                Turret.INSTANCE.setSide("red");
                sideSelected = true;
                startPose = new Pose(84, 10, Math.toRadians(90)); // Start Pose of our robot.
                score = new Pose(96, 86, Math.toRadians(270)); // Scoring Pose of our robot.
                readyPose0 = new Pose(89, 93, Math.toRadians(0));
                pickUpBalls0 = new Pose(116, 95, Math.toRadians(0));
                readyPose1 = new Pose(89, 71, Math.toRadians(0));
                pickUpBalls1 = new Pose(116, 71, Math.toRadians(0));
                readyPose2 = new Pose(89, 36, Math.toRadians(0));
                pickUpBalls2 = new Pose(116, 36, Math.toRadians(0));
                follower = Constants.createFollower(hardwareMap);

                buildPaths();
                follower.setStartingPose(startPose);
                telemetry.addData("Alliance: ","Red");
                telemetry.update();
            });



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









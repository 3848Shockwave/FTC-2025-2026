package org.firstinspires.ftc.teamcode.OpModes.Autonomous;
import static dev.nextftc.bindings.Bindings.button;

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

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
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
    private int sideSelected = -1;
    private int autoType = 0;
    Button x_button, y_button, a_button, b_button;
    public Command fireAll=null;

    private  Pose  startPose = new Pose(60, 10, Math.toRadians(90));
    private  Pose  score = new Pose(144 - 70, 86, Math.toRadians(270));; // Scoring Pose of our robot.
    private  Pose  readyPose0 = new Pose(65, 81, Math.toRadians(180));
    private  Pose pickUpBalls0 =  new Pose(38, 83, Math.toRadians(180));
    private  Pose readyPose1 = readyPose1 = new Pose(65, 59, Math.toRadians(180));
    private  Pose pickUpBalls1 =  pickUpBalls1 = new Pose(38, 59, Math.toRadians(180));
    private  Pose readyPose2 = new Pose(65, 36, Math.toRadians(180));
    private  Pose pickUpBalls2 = new Pose(25, 36, Math.toRadians(180));
    private  Pose  moveAutostartPose = new Pose(60, 10, Math.toRadians(90));
    private  Pose  moveAutoPose = new Pose(60, 10, Math.toRadians(90));

    private Path scorePreload;
    private Path move;
    private PathChain moveToReady0,moveToPick0, moveBackToScore0, moveToReady1, moveToPick1, moveBackToScore1,
            moveToReady2, moveToPick2, moveBackToScore2;
    private Command runAutoCommands() {
        if(autoType==0)
        {
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
        else{
            return new SequentialGroup(
                    new FollowPath(move)
            );
        }

    }

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */

          if(autoType==1) {
              move = new Path(new BezierLine(moveAutostartPose, moveAutoPose));
              move.setLinearHeadingInterpolation(moveAutostartPose.getHeading(), moveAutoPose.getHeading());
          }
          if (autoType==0) {
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
      //  autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
//        if(ActiveOpMode.isStarted()) {
//            telemetry.addData("path state", pathState);
//            telemetry.addData("x", follower.getPose().getX());
//            telemetry.addData("y", follower.getPose().getY());
//            telemetry.addData("heading", follower.getPose().getHeading());
//            telemetry.update();
//        }
    }

    public void onInit() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
         x_button = button(() -> gamepad1.x);
         y_button = button(() -> gamepad1.y);
         a_button = button(() -> gamepad1.a);
         b_button = button(() -> gamepad1.b);

    }
    @Override
    public void onWaitForStart() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        if(!ActiveOpMode.isStarted()) {
            if (sideSelected == -1) {
                telemetry.addData("Select Alliance ", "Left Button for Blue, Right for Red");
                telemetry.update();
            }
            x_button.whenBecomesTrue(() -> {
                sideSelected = 0;
                telemetry.addData("Alliance: ", "Blue");
                telemetry.addData("Select Auto: ", "Top Button for Score Auto, Bottom for Move Auto");
                telemetry.update();
                y_button.whenBecomesTrue(() -> {
                    telemetry.addData("Alliance: ", "Blue");
                    telemetry.addData("Auto: ", "Score");
                    telemetry.update();
                    autoType = 0;
                    //Turret.INSTANCE.setSide("blue");
                    startPose = new Pose(60, 10, Math.toRadians(90)); // Start Pose of our robot.
                    score = new Pose(70, 86, Math.toRadians(270)); // Scoring Pose of our robot.
                    readyPose0 = new Pose(65, 81, Math.toRadians(180));
                    pickUpBalls0 = new Pose(36, 83, Math.toRadians(180));
                    readyPose1 = new Pose(65, 59, Math.toRadians(180));
                    pickUpBalls1 = new Pose(38, 59, Math.toRadians(180));
                    readyPose2 = new Pose(65, 36, Math.toRadians(180));
                    pickUpBalls2 = new Pose(25, 36, Math.toRadians(180));
                    follower = Constants.createFollower(hardwareMap);
                    follower.setStartingPose(startPose);
                });
                a_button.whenBecomesTrue(() -> {
                   // Turret.INSTANCE.setSide("blue");
                    autoType = 1;
                    telemetry.addData("Alliance: ", "Blue");
                    telemetry.addData("Auto: ", "Move");
                    telemetry.update();
                    moveAutostartPose = new Pose(20, 128, Math.toRadians(315));
                    moveAutoPose = new Pose(44, 110, Math.toRadians(92));
                    follower = Constants.createFollower(hardwareMap);

                    follower.setStartingPose(moveAutostartPose);
                });


            });

            b_button.whenBecomesTrue(() ->
            {

                sideSelected = 1;
                telemetry.addData("Alliance: ", "Red");
                telemetry.addData("Select Auto: ", "Top Button for Score Auto, Bottom for Move Auto");
                telemetry.update();
                y_button.whenBecomesTrue(() -> {
                    autoType = 0;
                   // Turret.INSTANCE.setSide("red");
                    telemetry.addData("Alliance: ", "Red");
                    telemetry.addData("Auto: ", "Score");
                    telemetry.update();
                    startPose = new Pose(84, 10, Math.toRadians(90)); // Start Pose of our robot.
                    score = new Pose(96, 86, Math.toRadians(270)); // Scoring Pose of our robot.
                    readyPose0 = new Pose(89, 93, Math.toRadians(0));
                    pickUpBalls0 = new Pose(116, 95, Math.toRadians(0));
                    readyPose1 = new Pose(89, 71, Math.toRadians(0));
                    pickUpBalls1 = new Pose(116, 71, Math.toRadians(0));
                    readyPose2 = new Pose(89, 36, Math.toRadians(0));
                    pickUpBalls2 = new Pose(116, 36, Math.toRadians(0));
                    follower = Constants.createFollower(hardwareMap);

                    follower.setStartingPose(startPose);
                });
                a_button.whenBecomesTrue(() ->
                {
                    //Turret.INSTANCE.setSide("red");
                    autoType = 1;
                    telemetry.addData("Alliance: ", "Red");
                    telemetry.addData("Auto: ", "Move");
                    telemetry.update();
                    moveAutostartPose = new Pose(124, 128, Math.toRadians(225));
                    moveAutoPose = new Pose(78, 110, Math.toRadians(92));
                    follower = Constants.createFollower(hardwareMap);

                    follower.setStartingPose(moveAutostartPose);
                });
            });


        }
    }

    /**
     * This method is called once at the init of the OpMode.
     **/


    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void onStartButtonPressed() {
        buildPaths();
        setPathState(0);
        runAutoCommands().schedule();
        telemetry.addData("Auto Started", "");
        telemetry.update();
    }
}









package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import static dev.nextftc.bindings.Bindings.button;
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
import org.firstinspires.ftc.teamcode.Subsystems.MySubsystemGroup; // [新增] 导入
import org.firstinspires.ftc.teamcode.Subsystems.Sort;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Auto (Configurable & Vision)", preselectTeleOp="TeleOp Program")
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

    private PathChain moveToScore,load1,score2,ready2,score3; //blueFarSideScore
    private PathChain moveBLF;//blueFarSideMove
    private PathChain moveRF;//redFarSideMove
    private PathChain moveToScoreRF, loadRF1, scoreRF1, loadRF2, scoreRF2; //redFarSideScore

    private PathChain ObeliskMove,Shoot1,Collect1,Shoot2,Collect2,Shoot3;//blueGoalSideScore
    private PathChain ObeliskMoveRG, Shoot1RG; // redGoalSideMove
    private PathChain ObeliskMoveRM, Shoot1RM; //BlueGoalSideMove
    private PathChain MoveObeliskR, Shoot1R, Collect1R, Shoot2R, Collect2R, Shoot3R;//redGoalSideScore


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
        Drawing.init();
        follower = follower();
        Sort.INSTANCE.setAutoModeIsEnabled(false);
        RobotConfig.robotStateTracker = robotStateTracker;
        // follower() = Constants.createfollower()(hardwareMap);
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
            if(confirmed&&!readyToGo){
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
                       buildBlueGoalSideScorePaths();
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
                RobotConfig.autonomousStartEndPoses = isScore ?  RobotConfig.AutonomousStartEndPoses.GOALSIDEBLUESCORE :  RobotConfig.AutonomousStartEndPoses.GOALSIDEBLUEMOVE ;

            }
            RobotConfig.alliance = RobotConfig.Alliance.BLUE;
        } else { // RED
            if (isFar) {
                selectedMode = isScore ? AutoMode.RED_FAR_SIDE_SCORE : AutoMode.RED_FAR_SIDE_MOVE;
                RobotConfig.autonomousStartEndPoses = isScore ?  RobotConfig.AutonomousStartEndPoses.FARSIDEREDSCORE :  RobotConfig.AutonomousStartEndPoses.FARSIDEREDMOVE ;

            } else { // GOAL
                selectedMode = isScore ? AutoMode.RED_GOAL_SIDE_SCORE : AutoMode.RED_GOAL_SIDE_MOVE;
                RobotConfig.autonomousStartEndPoses = isScore ?  RobotConfig.AutonomousStartEndPoses.GOALSIDEREDSCORE :  RobotConfig.AutonomousStartEndPoses.GOALSIDEREDMOVE ;
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
                runBlueGoalSideScoreCommands().schedule();
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
//        telemetry.addData("Pose X", follower.getPose().getX());
//        telemetry.addData("Pose Y", follower.getPose().getY());
//        telemetry.addData("Pose Heading", Math.toDegrees(follower.getPose().getHeading()));
//        telemetry.addData("Starting Pose", RobotConfig.autonomousStartEndPoses.getStartPose());
//        Drawing.drawDebug(follower);
    }

    @Override
    public void onStop() {
        Turret.INSTANCE.setManualControl(false);
        RobotConfig.finalMeasuredPose = follower.getPose();
        RobotConfig.finalMeasuredSpindexPosition = Sort.INSTANCE.getServoPosition();
        RobotConfig.finalMeasuredColors = MySubsystemGroup.INSTANCE.getTargetColor();
        super.onStop();
    }


    //  RED GOAL SIDE MOVE
    private void buildRedGoalMovingPaths() {
        ObeliskMoveRG = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(119.269, 127.047),

                                new Pose(85.119, 100.188)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(270))

                .build();

        Shoot1RG = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(85.119, 100.188),

                                new Pose(90.648, 116.139)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(200))

                .build();
    }

    private Command runRedGoalMovingCommands() {
        return new SequentialGroup(
                new InstantCommand(() -> {
                    Turret.INSTANCE.setManualControl(true);
                    Turret.INSTANCE.setManualAnglePower(5,0);
                }),
                new FollowPath(ObeliskMoveRG),
                MySubsystemGroup.INSTANCE.detectTargetColorArray.thenWait(3),
                new InstantCommand(() -> {
                    Turret.INSTANCE.setManualAnglePower(25, 0);
                }),
                new FollowPath(Shoot1RG));
    }

    // BLUE GOAL SIDE MOVE
    private void buildBlueGoalMovingPaths() {
        ObeliskMoveRM = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(20.742, 123.457),

                                new Pose(55.983, 105.756)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(270))

                .build();

        Shoot1RM = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.983, 105.756),

                                new Pose(50.343, 115.939)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(320))

                .build();
    }

    private Command runBlueGoalMovingCommands() {
        return new SequentialGroup(
                new InstantCommand(() -> {
                    Turret.INSTANCE.setManualControl(true);
                    Turret.INSTANCE.setManualAnglePower(45,0);
                }),
                new FollowPath(ObeliskMoveRM),
                new Delay(3),
                new InstantCommand(() -> {
                    MySubsystemGroup.INSTANCE.detectTargetColorArray.thenWait(2);
                }),
                new InstantCommand(()-> {
                    Turret.INSTANCE.setManualAnglePower(25, 0);
                }),
                new FollowPath(Shoot1RM));

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
        return new SequentialGroup(  new InstantCommand(()->{
            Turret.INSTANCE.setManualControl(true);
            Turret.INSTANCE.setManualAnglePower(55,600);
        }),
                MySubsystemGroup.INSTANCE.detectTargetColorArray.thenWait(2),
                new FollowPath(moveBLF).and(new InstantCommand(()->{
                    Turret.INSTANCE.setManualAnglePower(35,600);
                })));
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
        return new SequentialGroup(  new InstantCommand(()->{
            Turret.INSTANCE.setManualControl(true);
            Turret.INSTANCE.setManualAnglePower(10,600);
        }),
                MySubsystemGroup.INSTANCE.detectTargetColorArray.thenWait(2),
                new FollowPath(moveRF).and(new InstantCommand(()->{
                    Turret.INSTANCE.setManualAnglePower(35,600);
                })));
    }


    // ==================== RED FAR SIDE (SCORE) ====================
    private void buildRedFarSideScorePaths() {
        moveToScoreRF = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(84.000, 8.500),

                                new Pose(86.508, 83.717)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(225))

                .build();

        loadRF1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(86.508, 83.717),
                                new Pose(94.073, 57.325),
                                new Pose(116.590, 56)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0),.2).setVelocityConstraint(15)

                .build();

        scoreRF1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(116.590, 57),

                                new Pose(86.194, 83.832)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))

                .build();

        loadRF2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(86.194, 83.832),

                                new Pose(114.3, 82.864)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0),.1).setVelocityConstraint(15)

                .build();

        scoreRF2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(114.3, 82.864),

                                new Pose(86.435, 103.670)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))

                .build();
        telemetry.addLine("Paths Built");
        telemetry.update();
    }

    private Command runRedFarSideScoreCommands() {
        return new SequentialGroup(
                new InstantCommand(()->{
                    Turret.INSTANCE.setManualControl(true);
                    Sort.INSTANCE.restartScissor();
                    Turret.INSTANCE.setManualAnglePower(15,100);
                }),

                MySubsystemGroup.INSTANCE.detectTargetColorArray.thenWait(.6),
                new FollowPath(moveToScoreRF).and(new InstantCommand(()->{
                            Turret.INSTANCE.setManualAnglePower(45,1325);
                        })
                ).thenWait(.5),
                MySubsystemGroup.INSTANCE.shootInPattern,
                new Delay(4.5),

                new FollowPath(loadRF1).and(
                        new InstantCommand(()->{
                            Turret.INSTANCE.setManualAnglePower(35,600);
                            Sort.INSTANCE.setAutoModeIsEnabled(true);
                            intake.setPower(1.0);
                        })
                ).thenWait(.5),

                new FollowPath(scoreRF1).and(
                        new InstantCommand(()->{
                            Turret.INSTANCE.setManualAnglePower(42.5,1250);

                        })
                ),new InstantCommand(()->{
                    Sort.INSTANCE.setAutoModeIsEnabled(false);

                }

                ).thenWait(.5)
                ,MySubsystemGroup.INSTANCE.shootInPattern,
                new Delay(4.5),

                new FollowPath(loadRF2).and(
                        new InstantCommand(()->{
                            Sort.INSTANCE.setAutoModeIsEnabled(true);
                            Turret.INSTANCE.setManualAnglePower(35,600);
                            intake.setPower(1.0);
                        })),

                new FollowPath(scoreRF2).and(new InstantCommand(()->{
                    Turret.INSTANCE.setManualAnglePower(50,1250);

                })).then(new InstantCommand(()->{
                    Sort.INSTANCE.setAutoModeIsEnabled(false);

                })).thenWait(.5)
                ,MySubsystemGroup.INSTANCE.shootInPattern
                        .thenWait(4.5).then(
                        new InstantCommand(()->{
                            Turret.INSTANCE.setManualAnglePower(25,600);

                        }))

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
                                new Pose(23.107, 59.887)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180),.4).setVelocityConstraint(10)

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

                                new Pose(23.836, 83.855)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180),.2).setVelocityConstraint(10)

                    .build();


        score3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(21.836, 83.855),

                                new Pose(47.670, 105.450)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))

                    .build();

        telemetry.addLine("Paths Built");
        telemetry.update();
    }

    private Command runBlueFarSideScoreCommands() {
        return new SequentialGroup(
                new InstantCommand(()->{
                    Turret.INSTANCE.setManualControl(true);
                    Sort.INSTANCE.restartScissor();
                    Turret.INSTANCE.setManualAnglePower(35,600);
                }),

                MySubsystemGroup.INSTANCE.detectTargetColorArray.thenWait(2),
                new FollowPath(moveToScore).and(new InstantCommand(()->{
                    Turret.INSTANCE.setManualAnglePower(36,1400);
                })
                ).thenWait(1),MySubsystemGroup.INSTANCE.shootInPattern,
                new Delay(4),

                new FollowPath(load1).and(
                        new InstantCommand(()->{
                            Sort.INSTANCE.setAutoModeIsEnabled(true);
                            Turret.INSTANCE.setManualAnglePower(20,600);
                            intake.setPower(1.0);
                        })
                ),
                new FollowPath(score2).and(
                        new InstantCommand(()->{
                            Sort.INSTANCE.setAutoModeIsEnabled(false);
                            Turret.INSTANCE.setManualAnglePower(32,1400);
                            intake.setPower(1.0);
                        })
                ).thenWait(1),MySubsystemGroup.INSTANCE.shootInPattern.thenWait(4),

                new FollowPath(ready2).and(
                        new InstantCommand(()->{
                            Sort.INSTANCE.setAutoModeIsEnabled(true);
                            Turret.INSTANCE.setManualAnglePower(15,600);
                            intake.setPower(1.0);
                })),

                new FollowPath(score3).and(new InstantCommand(()->{
                    Sort.INSTANCE.setAutoModeIsEnabled(false);
                    Turret.INSTANCE.setManualAnglePower(32,1400);
                })),MySubsystemGroup.INSTANCE.shootInPattern
        );

    }

    // ==================== RED GOAL SIDE (SCORE) ====================
    private void buildRedGoalSideScorePaths() {
        MoveObeliskR = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(121.994, 123.590),

                                new Pose(96.488, 96.233)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(270))

                .build();

        Shoot1R = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(96.488, 96.233),

                                new Pose(96.670, 96.288)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(230))

                .build();

        Collect1R = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(96.670, 96.288),
                                new Pose(91.766, 85.489),
                                new Pose(118.3, 82.864)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(360),.15).setVelocityConstraint(10)

                .build();

        Shoot2R = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(118.3, 82.864),

                                new Pose(96.812, 96.524)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(230))

                .build();

        Collect2R = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(96.812, 96.524),
                                new Pose(73.093, 63.458),
                                new Pose(118.590, 57)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(360),.15).setVelocityConstraint(10)

                .build();

        Shoot3R = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(118.590, 55.5),

                                new Pose(96.310, 110.825)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(230))

                .build();

        telemetry.addLine("Paths Built");
        telemetry.update();
    }

    private Command runRedGoalSideScoreCommands() {
        return new SequentialGroup(
                new InstantCommand(()->{
                    Turret.INSTANCE.setManualControl(true);
                    Sort.INSTANCE.restartScissor();
                    Turret.INSTANCE.setManualAnglePower(15,600);
                }),
                new FollowPath(MoveObeliskR),
                MySubsystemGroup.INSTANCE.detectTargetColorArray.thenWait(1.0),
                new FollowPath(Shoot1R).and(
                        new InstantCommand(()->{
                            Turret.INSTANCE.setManualAnglePower(50,1250);
                        })
                ),MySubsystemGroup.INSTANCE.shootInPattern,
                new Delay(5),
                new FollowPath(Collect1R).and(
                        new InstantCommand(()->{
                            Turret.INSTANCE.setManualAnglePower(35,600);
                            Sort.INSTANCE.setAutoModeIsEnabled(true);
                            intake.setPower(1.0);

                        })
                ).thenWait(.5),

                new FollowPath(Shoot2R).and(
                        new InstantCommand(()->{
                            Sort.INSTANCE.setAutoModeIsEnabled(false);
                            Turret.INSTANCE.setManualAnglePower(50,1250);
                            // intake.setPower(0);
                        })
                )
                ,MySubsystemGroup.INSTANCE.shootInPattern,
                new Delay(5),
                new FollowPath(Collect2R).and(
                        new InstantCommand(()->{
                            Sort.INSTANCE.setAutoModeIsEnabled(true);
                            Turret.INSTANCE.setManualAnglePower(35,600);
                            intake.setPower(1.0);

                        })
                ),
                new FollowPath(Shoot3R).and(new InstantCommand(()->{

                    Turret.INSTANCE.setManualAnglePower(65,1200);
                    // intake.setPower(0.0);
                })).then(new InstantCommand(()->{
                    Sort.INSTANCE.setAutoModeIsEnabled(false);})).thenWait(.1),
                MySubsystemGroup.INSTANCE.shootInPattern,
                new Delay(5).then(
                        new InstantCommand(()->{
                            Turret.INSTANCE.setManualAnglePower(25,600);
                            intake.setPower(0.0);
                        }))

        );
    }
    // ==================== RED GOAL SIDE (SCORE) ====================
    private void buildBlueGoalSideScorePaths() {
        ObeliskMove = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(20.742, 123.457),

                                new Pose(46.825, 96.632)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(250))

                .build();

        Shoot1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(46.825, 96.632),

                                new Pose(46.753, 96.593)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(315))

                .build();

        Collect1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(46.753, 96.593),
                                new Pose(48.766, 82.892),
                                new Pose(30, 82.5)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180),.15).setVelocityConstraint(10)

                .build();

        Shoot2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(19.460, 84.366),

                                new Pose(46.798, 96.662)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(320))

                .build();

        Collect2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(46.798, 96.662),
                                new Pose(61.561, 65.724),
                                new Pose(30, 56.0)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(320), Math.toRadians(180),.2).setVelocityConstraint(10)

                .build();

        Shoot3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(19.072, 59.740),

                                new Pose(46.648, 106.537)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))

                .build();

        telemetry.addLine("Paths Built");
        telemetry.update();
    }

    private Command runBlueGoalSideScoreCommands() {
        return new SequentialGroup(
                new InstantCommand(()->{
                    Turret.INSTANCE.setManualControl(true);
                    Sort.INSTANCE.restartScissor();
                    Turret.INSTANCE.setManualAnglePower(35,600);
                }),

                new FollowPath(ObeliskMove),
                MySubsystemGroup.INSTANCE.detectTargetColorArray.thenWait(1.0),


                new FollowPath(Shoot1).and(
                        new InstantCommand(()->{
                            Turret.INSTANCE.setManualAnglePower(50,1250);
                        })
                ),new Delay (1),
                MySubsystemGroup.INSTANCE.shootInPattern,
                new Delay(4),


                new FollowPath(Collect1).and(
                        new InstantCommand(()->{
                            Turret.INSTANCE.setManualAnglePower(35,600);
                            Sort.INSTANCE.setAutoModeIsEnabled(true);
                            intake.setPower(1.0);

                        })
                ).thenWait(1),

                new FollowPath(Shoot2).and(
                        new InstantCommand(()->{
                            Sort.INSTANCE.setAutoModeIsEnabled(false);
                            Turret.INSTANCE.setManualAnglePower(50,1250);
                           // intake.setPower(0);
                        })
                ).thenWait(1)
                ,MySubsystemGroup.INSTANCE.shootInPattern,
                new Delay(4),



                new FollowPath(Collect2).and(
                        new InstantCommand(()->{
                            Sort.INSTANCE.setAutoModeIsEnabled(true);
                            Turret.INSTANCE.setManualAnglePower(35,600);
                            intake.setPower(1.0);

                        })
                ),

                new FollowPath(Shoot3).and(new InstantCommand(()->{
                    Sort.INSTANCE.setAutoModeIsEnabled(false);
                    Turret.INSTANCE.setManualAnglePower(50,1250);
                   // intake.setPower(0.0);
                })),
                new Delay (1),
                MySubsystemGroup.INSTANCE.shootInPattern,
                new Delay(4.5).then(
                        new InstantCommand(()->{
                            Turret.INSTANCE.setManualAnglePower(25,600);
                            intake.setPower(0.0);
                        }))

        );
    }
}
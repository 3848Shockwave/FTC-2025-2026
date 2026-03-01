package org.firstinspires.ftc.teamcode.OpModes;

import static dev.nextftc.bindings.Bindings.button;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Helpers.RobotConfig;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.RobotStateTracker;
import org.firstinspires.ftc.teamcode.Subsystems.MySubsystemGroup;
import org.firstinspires.ftc.teamcode.Subsystems.PTO;
import org.firstinspires.ftc.teamcode.Subsystems.Sort;
import org.firstinspires.ftc.teamcode.Subsystems.Sort.Color;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.impl.MotorEx;


@Configurable
@TeleOp(name = "TeleOp Program", group = "Production")
public class TeleOpProgram extends NextFTCOpMode {

    public static RobotStateTracker antiCrazy;
    public static double newVelocity = 1345;

    public static double power = 1.0;
    public static double xOffset = 0.0;
    private final boolean launchToggle = false;
    private boolean sideSelected = false;
    MotorEx intake = new MotorEx("intakeMotor").brakeMode();

    private final MotorEx leftBack = new MotorEx("back_left");
    private final MotorEx rightBack = new MotorEx("back_right");
    private final MotorEx leftFront = new MotorEx("front_left");
    private final MotorEx rightFront = new MotorEx("front_right");
    Button x_button, y_button, a_button, b_button;
    Pose startPose = null;
    private boolean motorToggle = false;
    private TelemetryManager telemetryManager;
    public static double Rkp = 0.004;
    public static double Rkd = 0.00026;
    public static double Rki = 0.004;
    public static double Rkf = 0.0000275;
public static double Lkp =0.0004;
    public static double Lki =  0.004;
    public static double Lkd =  0.0088;
    public static double Lkf = 0.000452;
    public static double speed =1345;
    public boolean PTOEngaged = false;
    public boolean intakeOn = false;
    private boolean overrideUpdatePose = false;
    boolean isNotfull = false;
    double lastLoopTime = 0;



    public TeleOpProgram() {
        addComponents(
               // new SubsystemComponent(Sort.INSTANCE),
               // new SubsystemComponent(Turret.INSTANCE),
                new SubsystemComponent(PTO.INSTANCE),
                new SubsystemComponent(MySubsystemGroup.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    @Override
    public void onInit() {
        Turret.INSTANCE.setManualControl(false);
        Sort.INSTANCE.setAutoModeIsEnabled(false);
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        if(RobotConfig.autonomousStartEndPoses != null){
            startPose = RobotConfig.autonomousStartEndPoses.getEndPose();
            telemetryManager.addData("Start selected: ", RobotConfig.autonomousStartEndPoses.name());
            sideSelected = true;
            antiCrazy = RobotConfig.robotStateTracker;
            Turret.INSTANCE.initLimelightSystem();
        }
        if(RobotConfig.finalMeasuredColors!=null){
            MySubsystemGroup.INSTANCE.setTargetColor(RobotConfig.finalMeasuredColors);
        }


        telemetryManager.update(telemetry);
        if (RobotConfig.autonomousStartEndPoses == null) {
            x_button = button(() -> gamepad1.x);
            y_button = button(() -> gamepad1.y);
            a_button = button(() -> gamepad1.a);
            b_button = button(() -> gamepad1.b);
            telemetryManager.addData("You need to select an Auto/Alliance",
                    "\nPress SQUARE for BLUE FAR SIDE autonomous" +
                            "\nPress TRIANGLE for BLUE GOAL SIDE autonomous" +
                            "\nPress X for RED FAR SIDE autonomous" +
                            "\nPress CIRCLE for RED GOAL SIDE autonomous");
            telemetryManager.update(telemetry);
            if (ActiveOpMode.opModeInInit()&&!sideSelected) {
                x_button.whenBecomesTrue(() -> {
                    if (ActiveOpMode.opModeInInit()&&!sideSelected) {
                        RobotConfig.alliance = RobotConfig.Alliance.BLUE;
                        RobotConfig.autonomousStartEndPoses = RobotConfig.AutonomousStartEndPoses.FARSIDEBLUEMOVE;
                        telemetryManager.addData("Start Selected:", " BLUE FAR SIDE autonomous");
                        telemetryManager.update(telemetry);
                        sideSelected = true;
                        RobotConfig.robotStateTracker = new RobotStateTracker();
                        antiCrazy = RobotConfig.robotStateTracker;
                        Turret.INSTANCE.initLimelightSystem();
                    }
                });
                y_button.whenBecomesTrue(() -> {
                    if (ActiveOpMode.opModeInInit()&&!sideSelected) {
                        RobotConfig.alliance = RobotConfig.Alliance.BLUE;
                        RobotConfig.autonomousStartEndPoses = RobotConfig.AutonomousStartEndPoses.GOALSIDEBLUESCORE;
                        telemetryManager.addData("Start Selected:", " BLUE GOAL SIDE autonomous");
                        telemetryManager.update(telemetry);
                        sideSelected = true;
                        RobotConfig.robotStateTracker = new RobotStateTracker();
                        antiCrazy = RobotConfig.robotStateTracker;
                        Turret.INSTANCE.initLimelightSystem();
                    }
                });
                a_button.whenBecomesTrue(() -> {
                    if (ActiveOpMode.opModeInInit()&&!sideSelected) {
                        RobotConfig.alliance = RobotConfig.Alliance.RED;
                        RobotConfig.autonomousStartEndPoses = RobotConfig.AutonomousStartEndPoses.FARSIDEREDMOVE;
                        telemetryManager.addData("Start Selected:", " RED FAR SIDE autonomous");
                        telemetryManager.update(telemetry);
                        sideSelected = true;
                        RobotConfig.robotStateTracker = new RobotStateTracker();
                        antiCrazy = RobotConfig.robotStateTracker;
                        Turret.INSTANCE.initLimelightSystem();
                    }
                });
                b_button.whenBecomesTrue(() -> {
                    if (ActiveOpMode.opModeInInit()&&!sideSelected) {
                        RobotConfig.alliance = RobotConfig.Alliance.RED;
                        RobotConfig.autonomousStartEndPoses = RobotConfig.AutonomousStartEndPoses.GOALSIDEREDMOVE;
                        telemetryManager.addData("Start Selected:", " RED GOAL SIDE autonomous");
                        telemetryManager.update(telemetry);
                        sideSelected = true;
                        RobotConfig.robotStateTracker = new RobotStateTracker();
                        antiCrazy = RobotConfig.robotStateTracker;
                        Turret.INSTANCE.initLimelightSystem();
                    }
                });
            }
        }

        if(sideSelected) {
            if(RobotConfig.finalMeasuredPose!=null) {
                antiCrazy.updateLastPose(RobotConfig.finalMeasuredPose);
            }
            follower().setPose(startPose);
        }


        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void onStartButtonPressed() {
   //     Turret.INSTANCE.RunTurret.schedule();

        Sort.INSTANCE.updateServo();
        Sort.INSTANCE.restartScissor();



        //=================== GAMEPAD 1 CONTROLS =================//

        //Intake IN
                    Button dpad_up = button(() -> gamepad1.dpad_up).whenBecomesTrue(() -> {
                        motorToggle = !motorToggle;

                        for(Color color: Sort.INSTANCE.getColorArray()){
                            if (color == Color.EMPTY) {
                                isNotfull = true;
                                break;
                            }
                        }

                        if (motorToggle&&isNotfull) {
                            intake.setPower(1);
                            intakeOn = true;
                        } else {
                            intake.setPower(0);
                            intakeOn = false;
                        }
                    });

        //Intake REVERSE
                    Button dpad_down = button(() -> gamepad1.dpad_down).whenBecomesTrue(() -> {
                        motorToggle = !motorToggle;
                        if (motorToggle) {
                            intake.setPower(-1);
                        } else {
                            intake.setPower(0);
                        }
                    });


        //Detect Colors Manually
                    Button dpad_right = button(()->gamepad1.dpad_right)
                            .whenBecomesTrue(MySubsystemGroup.INSTANCE.detectTargetColorArray);


        // Triple Launch
                    Button x = button(() -> gamepad1.x)
                            .whenBecomesTrue(() -> {
                                MySubsystemGroup.INSTANCE.tripleLaunch.schedule();
                            });
        // Normal  Shoot
                    Button y_button = button(() -> gamepad1.y)
                            .whenBecomesTrue(Sort.INSTANCE.pushBallAndBack);
        // Closest Shoot
                    Button b_button = button(() -> gamepad1.b)
                            .whenBecomesTrue(()->{Sort.INSTANCE.shootClosestBall().schedule();});
        // Shoot In Pattern
                    Button a_button = button(() -> gamepad1.a)
                            .whenBecomesTrue(MySubsystemGroup.INSTANCE.shootInPattern);
        // Shoot Purple
                    Button right_trigger = button(() -> gamepad1.right_trigger > 0.5)
                            .whenBecomesTrue(()->{
                                Sort.INSTANCE.shootPurp.schedule();
                            });
        // Shoot Green
                    Button left_trigger = button(() -> gamepad1.left_trigger > 0.5)
                            .whenBecomesTrue(()->{
                                Sort.INSTANCE.shootGreen.schedule();
                            });
        // Cycle Right
                    Button left_bumper = button(() -> gamepad1.left_bumper)
                            .whenBecomesTrue(
                                    Sort.INSTANCE.cycleRight
                            );
        // Cycle Left
                    Button right_bumper = button(() -> gamepad1.right_bumper)
                            .whenBecomesTrue(Sort.INSTANCE.cycleLeft
                            );




//        Button dpad_left = button(()->gamepad1.dpad_left)
//                .whenBecomesTrue(()->{
//                    overrideUpdatePose = true;
//                    PedroComponent.follower().setPose(new Pose(antiCrazy.getLastMeasuredPose().getX(),antiCrazy.getLastMeasuredPose().getY(),90));
//
//                });







        Button PTOEngage = button(() -> gamepad2.y)
                .whenBecomesTrue(()->{
                    follower().breakFollowing();
                    PTO.INSTANCE.engage.schedule();
                    PTOEngaged = true;
                });



        follower().startTeleopDrive();
        DriverControlledCommand  driverControlled = null;
        if(RobotConfig.alliance == RobotConfig.Alliance.BLUE) {
               driverControlled =new PedroDriverControlled(
                    Gamepads.gamepad1().leftStickY(),
                    Gamepads.gamepad1().leftStickX(),
                    Gamepads.gamepad1().rightStickX().negate(),
                    false
            );
        }
            if(RobotConfig.alliance == RobotConfig.Alliance.RED){
                driverControlled = new PedroDriverControlled(
                        Gamepads.gamepad1().leftStickY().negate(),
                        Gamepads.gamepad1().leftStickX().negate(),
                        Gamepads.gamepad1().rightStickX().negate(),
                        false
                );
            }

        driverControlled.schedule();
    }

    @Override
    public void onUpdate() {
        ElapsedTime loopTimer = new ElapsedTime();
        double start = loopTimer.milliseconds();
        if(intakeOn&&Sort.INSTANCE.isSpindexStable()){
            if(Sort.INSTANCE.getColorArray()[2]!= Color.EMPTY) {
                if (Sort.INSTANCE.getColorArray()[0] == Color.EMPTY) {
                    Sort.INSTANCE.cycleLeft.schedule();
                } else if (Sort.INSTANCE.getColorArray()[1] == Color.EMPTY) {
                    Sort.INSTANCE.cycleRight.schedule();
                }
            }
        }

        if(PTOEngaged) {
            if (gamepad2.left_trigger > 0.5) {
                leftBack.setPower(-gamepad2.left_trigger);
                leftFront.setPower(-gamepad2.left_trigger);
            }  else if(gamepad2.left_trigger < .5){
                leftBack.setPower(0.0);
            }
            if (gamepad2.right_trigger > .5) {
                rightBack.setPower(gamepad2.right_trigger);
                rightFront.setPower(gamepad2.right_trigger);
            } else if(gamepad2.right_trigger < .5){
                rightBack.setPower(0.0);
            }
        }
        if(antiCrazy.getLastMeasuredPose()!=null) {
            if (!follower().getPose().roughlyEquals(antiCrazy.getLastMeasuredPose(), 15)&&!overrideUpdatePose) {
                follower().setPose(antiCrazy.getLastMeasuredPose());
            } else {
                antiCrazy.updateLastPose(follower().getPose());
                if(overrideUpdatePose){
                    overrideUpdatePose = false;
                }
            }
        }
        else if(RobotConfig.finalMeasuredPose!=null&&ActiveOpMode.isStarted()){
            antiCrazy.updateLastPose(RobotConfig.finalMeasuredPose);
        }

        double turretWant = Turret.INSTANCE.getRealTurretPosition() + Turret.INSTANCE.calculatePosition();
        Turret.INSTANCE.setXoffset(xOffset);
        telemetryManager.addLine("-----------------------------");
        telemetryManager.addLine("General TeleOp Info:");
        telemetryManager.addLine("-----------------------------");
        telemetryManager.addData("Commands:", CommandManager.INSTANCE.snapshot());
        telemetryManager.addData("Side Selected: ", RobotConfig.alliance.name());
        telemetryManager.addData("Pose X & Y:", follower().getPose().getX()+" , "+follower().getPose().getY());

        telemetryManager.addLine("-----------------------------");
        telemetryManager.addLine("===== SORTING SYSTEM ======");
        telemetryManager.addLine("-----------------------------");
        if(Sort.INSTANCE.getColorArray() != null) {
            Color[] colors = Sort.INSTANCE.getColorArray();
            telemetryManager.addData("Colors indexed (Right/Left/Shoot)",
                    colors[0] + ", " + colors[1] + ", " + colors[2]
            );
        }
        if(MySubsystemGroup.INSTANCE.getTargetColor()!=null) {
            telemetryManager.addData("Target Colors: ", MySubsystemGroup.INSTANCE.getTargetColor()[0]+", "+ MySubsystemGroup.INSTANCE.getTargetColor()[1]+", "+ MySubsystemGroup.INSTANCE.getTargetColor()[2]);
        }
        telemetryManager.addData("Current Spindex Index (0-2)", Sort.INSTANCE.getCurrentIndex());
        telemetryManager.addData("Spindex Stability", Sort.INSTANCE.isSpindexStable());
        telemetryManager.addData("ScissorLift Staus",Sort.INSTANCE.isTouchPressed());


        telemetryManager.addLine("-----------------------------");
        telemetryManager.addLine("===== LAUNCH SYSTEM ======");
        telemetryManager.addLine("-----------------------------");
        telemetryManager.addData("Pipeline", Turret.INSTANCE.limelightProcessing.getCurrentPipeline());
        telemetryManager.addData("Limelight Status: ", Turret.INSTANCE.limelightProcessing.limelightTelemetry());
        double currentLoopTime = System.nanoTime();
        double loopFrequency = 1000000000 / (currentLoopTime - lastLoopTime);
        lastLoopTime = currentLoopTime;

        telemetry.addData("Loop Frequency", "%.0f Hz", loopFrequency);

        telemetryManager.update(telemetry);
    }
}
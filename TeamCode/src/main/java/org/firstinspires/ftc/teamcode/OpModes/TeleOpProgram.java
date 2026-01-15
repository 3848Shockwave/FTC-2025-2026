package org.firstinspires.ftc.teamcode.OpModes;

import static dev.nextftc.bindings.Bindings.button;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import android.animation.RectEvaluator;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.Subsystems.Helpers.RobotConfig;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.RobotStateTracker;
import org.firstinspires.ftc.teamcode.Subsystems.MySubsystemGroup;
import org.firstinspires.ftc.teamcode.Subsystems.PTO;
import org.firstinspires.ftc.teamcode.Subsystems.Sort;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Sort.Color; // Updated Import

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.groups.SequentialGroup;
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


    public TeleOpProgram() {
        addComponents(
                new SubsystemComponent(Sort.INSTANCE),
                new SubsystemComponent(Turret.INSTANCE),
                new SubsystemComponent(PTO.INSTANCE),
                //new SubsystemComponent(MySubsystemGroup.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    @Override
    public void onInit() {

        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        if(RobotConfig.autonomousStartEndPoses != null){
            startPose = RobotConfig.autonomousStartEndPoses.getEndPose();
            telemetryManager.addData("Start selected: ", RobotConfig.autonomousStartEndPoses.name());
            sideSelected = true;
            antiCrazy = RobotConfig.robotStateTracker;
            Turret.INSTANCE.initLimelightSystem();
        }

        telemetryManager.update(telemetry);
        if (RobotConfig.autonomousStartEndPoses == null) {
            x_button = button(() -> gamepad1.x);
            y_button = button(() -> gamepad1.y);
            a_button = button(() -> gamepad1.a);
            b_button = button(() -> gamepad1.b);
            telemetryManager.addData("You need to select an Auto/Alliance",
                    "\nPress X for BLUE FAR SIDE autonomous" +
                            "\nPress Y for BLUE GOAL SIDE autonomous" +
                            "\nPress A for RED FAR SIDE autonomous" +
                            "\nPress B for RED GOAL SIDE autonomous");
            telemetryManager.update(telemetry);

            if (ActiveOpMode.opModeInInit()) {
                x_button.whenBecomesTrue(() -> {
                    RobotConfig.alliance = RobotConfig.Alliance.BLUE;
                    RobotConfig.autonomousStartEndPoses = RobotConfig.AutonomousStartEndPoses.FARSIDEBLUE;
                    telemetryManager.addData("Start Selected:", " BLUE FAR SIDE autonomous");
                    telemetryManager.update(telemetry);
                    sideSelected=true;
                    RobotConfig.robotStateTracker = new RobotStateTracker();
                    antiCrazy = RobotConfig.robotStateTracker;
                    Turret.INSTANCE.initLimelightSystem();
                });
                y_button.whenBecomesTrue(() -> {
                    RobotConfig.alliance = RobotConfig.Alliance.BLUE;
                    RobotConfig.autonomousStartEndPoses = RobotConfig.AutonomousStartEndPoses.GOALSIDEBLUE;
                    telemetryManager.addData("Start Selected:", " BLUE GOAL SIDE autonomous");
                    telemetryManager.update(telemetry);
                    sideSelected=true;
                    RobotConfig.robotStateTracker = new RobotStateTracker();
                    antiCrazy = RobotConfig.robotStateTracker;
                    Turret.INSTANCE.initLimelightSystem();
                });
                a_button.whenBecomesTrue(() -> {
                    RobotConfig.alliance = RobotConfig.Alliance.RED;
                    RobotConfig.autonomousStartEndPoses = RobotConfig.AutonomousStartEndPoses.FARSIDERED;
                    telemetryManager.addData("Start Selected:", " RED FAR SIDE autonomous");
                    telemetryManager.update(telemetry);
                    sideSelected=true;
                    RobotConfig.robotStateTracker = new RobotStateTracker();
                    antiCrazy = RobotConfig.robotStateTracker;
                    Turret.INSTANCE.initLimelightSystem();
                });
                b_button.whenBecomesTrue(() -> {
                    RobotConfig.alliance = RobotConfig.Alliance.RED;
                    RobotConfig.autonomousStartEndPoses = RobotConfig.AutonomousStartEndPoses.GOALSIDERED;
                    telemetryManager.addData("Start Selected:", " RED GOAL SIDE autonomous");
                    telemetryManager.update(telemetry);
                    sideSelected=true;
                    RobotConfig.robotStateTracker = new RobotStateTracker();
                    antiCrazy = RobotConfig.robotStateTracker;
                    Turret.INSTANCE.initLimelightSystem();
                });
            }
        }

        if(sideSelected) {
            if(RobotConfig.finalMeasuredPose!=null) {
                antiCrazy.updateLastPose(RobotConfig.finalMeasuredPose);
            }
            follower().setPose(startPose);
        }

        Turret.INSTANCE.resetRotateMotorPosition();
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void onStartButtonPressed() {
   //     Turret.INSTANCE.RunTurret.schedule();

        // 1. Move spindex Hardware
        Sort.INSTANCE.updateServo();
        Button dpad_up = button(() -> gamepad1.dpad_up).whenBecomesTrue(() -> {
            motorToggle = !motorToggle;
            if (motorToggle) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }
        });

        Button dpad_down = button(() -> gamepad1.dpad_down).whenBecomesTrue(() -> {
            motorToggle = !motorToggle;
            if (motorToggle) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }
        });

         Button x = button(() -> gamepad1.x)
                .whenBecomesTrue(()->{
                   new SequentialGroup(
                            Sort.INSTANCE.pushBallAndBack,
                            Sort.INSTANCE.cycleLeft,
                            Sort.INSTANCE.pushBallAndBack,
                            Sort.INSTANCE.cycleLeft,
                            Sort.INSTANCE.pushBallAndBack
                     ).schedule();
                });

        Button y_button = button(() -> gamepad1.y)
                .whenBecomesTrue(Sort.INSTANCE.pushBallAndBack);
        Button a_button = button(() -> gamepad1.a)
                .whenBecomesTrue(MySubsystemGroup.INSTANCE.shootInPattern);
        Button right_trigger = button(() -> gamepad1.right_trigger > 0.5)
                .whenBecomesTrue(()->{
                 Sort.INSTANCE.shootPurp.schedule();
                });
        Button left_trigger = button(() -> gamepad1.left_trigger > 0.5)
                .whenBecomesTrue(()->{
                    Sort.INSTANCE.shootGreen.schedule();
                });

        Button left_bumper = button(() -> gamepad1.left_bumper)
                .whenBecomesTrue(
                    Sort.INSTANCE.cycleLeft
                );

        Button right_bumper = button(() -> gamepad1.right_bumper)
                .whenBecomesTrue(Sort.INSTANCE.cycleRight
                );

        Button PTOEngage = button(() -> gamepad2.y)
                .whenBecomesTrue(()->{
                    follower().breakFollowing();
                    PTO.INSTANCE.engage.schedule();
                });

        Button PTOLeftSide =  button(() -> gamepad2.left_bumper);
        Button PTORightSide =  button(() -> gamepad2.right_bumper);

        PTOLeftSide.whenTrue(()->{
            leftBack.setPower(1.0);
            PTO.INSTANCE.engageL.schedule();
        });
        PTOLeftSide.whenBecomesFalse(()->leftBack.setPower(0.0));

        PTORightSide.whenTrue(()->{
            rightBack.setPower(-1.0);
            PTO.INSTANCE.engageR.schedule();
        });
        PTORightSide.whenBecomesFalse(()->rightBack.setPower(0.0));


        follower().startTeleopDrive();

        DriverControlledCommand driverControlled = new PedroDriverControlled(

                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().rightStickX().negate(),
                false
        );
        driverControlled.schedule();
    }

    @Override
    public void onUpdate() {
        if(antiCrazy.getLastMeasuredPose()!=null) {
            if (!follower().getPose().roughlyEquals(antiCrazy.getLastMeasuredPose(), 15)) {
                follower().setPose(antiCrazy.getLastMeasuredPose());
            } else {
                antiCrazy.updateLastPose(follower().getPose());
            }
        }
        else if(RobotConfig.finalMeasuredPose!=null&&ActiveOpMode.isStarted()){
            antiCrazy.updateLastPose(RobotConfig.finalMeasuredPose);
        }

        double turretWant = Turret.INSTANCE.getRealTurretPosition() + Turret.INSTANCE.calculatePosition();
        Turret.INSTANCE.setXoffset(xOffset);


        if(Sort.INSTANCE.getColorArray() != null) {
            Color[] colors = Sort.INSTANCE.getColorArray();
            telemetryManager.addData("=== SORTING SYSTEM ===", "");
            // Note: Index 2 might not update automatically in new logic unless specifically set
            telemetryManager.addData("Colors (R/L/Shoot)",
                    colors[0] + ", " + colors[1] + ", " + colors[2]
            );
        }
        telemetryManager.addData("Commands:", CommandManager.INSTANCE.snapshot());
        telemetryManager.addData("=== SORT SYSTEM ===", "");
        telemetryManager.addData("Current Index (0-2)", Sort.INSTANCE.getCurrentIndex());
        telemetryManager.addData("Servo Command Pos", Sort.INSTANCE.getServoPosition());

        // --- Turret Telemetry ---
        Turret.INSTANCE.getRotateEncoder().updateRotations();
        telemetryManager.addData("goalvel", newVelocity);
        telemetryManager.addData("turretMotorPosition", Turret.INSTANCE.getRealTurretPosition());
        telemetryManager.addData("TurretNextPosition", turretWant);

        telemetryManager.addData("=== ROTATION TRACKING ===", "");
        telemetryManager.addData("Rotations", Turret.INSTANCE.getRotateEncoder().getRotations());
        telemetryManager.addData("Position",Turret.INSTANCE.getRotateEncoder().getTotalDegrees());
        telemetryManager.addData("Calculate Position", Turret.INSTANCE.calculatePosition());
        telemetryManager.addData("Next Position", Turret.INSTANCE.getNextTurretPosition());
        telemetryManager.addData("Real Goal Position", Turret.INSTANCE.getControlSystemRotate().getGoal());




        // Turret.INSTANCE.rebuildControlSystem(Rkp,Rki,Rkd,Rkf,100);
        // --- Limelight Telemetry ---
        telemetryManager.addData("Pipeline", Turret.INSTANCE.limelightProcessing.getCurrentPipeline());
        telemetryManager.addData("Limelight Status", Turret.INSTANCE.limelightProcessing.limelightTelemetry());
        telemetryManager.addData("turretVelocity", Turret.INSTANCE.getTurretVelocity());
        Turret.INSTANCE.setTestSpeed(speed);

        telemetryManager.update(telemetry);
    }
}
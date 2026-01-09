package org.firstinspires.ftc.teamcode.OpModes;

import static dev.nextftc.bindings.Bindings.button;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Helpers.RobotConfig;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.RobotStateTracker;
import org.firstinspires.ftc.teamcode.Subsystems.Sort;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.bindings.Button;
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

    public static RobotStateTracker antiCrazy = new RobotStateTracker();
    public static double newVelocity = 0.0;
    //    public static double kp =0.004;
//    public static double ki = 0.004;
//    public static double kd =0.00026;
//    public static double kf = 0.0000275;
    //launcher
//private static double kp =0.008;
//    private static double ki =  0.9;
//    private static double kd =  0.000375;
//    private static double kf = 0.0005;
    public static double power = 1.0;
    public static double xOffset = 0.0;
    private final boolean launchToggle = false;
    MotorEx intake = new MotorEx("intakeMotor").brakeMode();
    Button x_button, y_button, a_button, b_button;
    Pose startPose = null;
    private boolean motorToggle = false;
    private TelemetryManager telemetryManager;


    public TeleOpProgram() {
        addComponents(
                new SubsystemComponent(Sort.INSTANCE),
                new SubsystemComponent(Turret.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    @Override
    public void onInit() {
        startPose = RobotConfig.autonomousStartEndPoses.getEndPose();
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetryManager.addData("Start selected: ", RobotConfig.autonomousStartEndPoses.name());
        telemetryManager.update(telemetry);
        if (startPose == null) {
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

                });
                y_button.whenBecomesTrue(() -> {
                    RobotConfig.alliance = RobotConfig.Alliance.BLUE;
                    RobotConfig.autonomousStartEndPoses = RobotConfig.AutonomousStartEndPoses.GOALSIDEBLUE;
                    telemetryManager.addData("Start Selected:", " BLUE GOAL SIDE autonomous");
                    telemetryManager.update(telemetry);

                });
                a_button.whenBecomesTrue(() -> {
                    RobotConfig.alliance = RobotConfig.Alliance.RED;
                    RobotConfig.autonomousStartEndPoses = RobotConfig.AutonomousStartEndPoses.FARSIDERED;
                    telemetryManager.addData("Start Selected:", " RED FAR SIDE autonomous");
                    telemetryManager.update(telemetry);

                });
                b_button.whenBecomesTrue(() -> {
                    RobotConfig.alliance = RobotConfig.Alliance.RED;
                    RobotConfig.autonomousStartEndPoses = RobotConfig.AutonomousStartEndPoses.GOALSIDERED;
                    telemetryManager.addData("Start Selected:", " RED GOAL SIDE autonomous");
                    telemetryManager.update(telemetry);

                });
            }
        }
        antiCrazy.updateLastPose(RobotConfig.finalMeasuredPose);
        follower().setPose(startPose);
        Turret.INSTANCE.resetRotateMotorPosition();
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Turret.INSTANCE.limelightProcessing.getLimelightStatus();
    }

    @Override
    public void onStartButtonPressed() {
        Button a_button = button(() -> gamepad1.a).whenBecomesTrue(() ->
        {
            motorToggle = !motorToggle;
            if (motorToggle) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }
        });

        Button b_button = button(() -> gamepad1.b).whenBecomesTrue(() ->
        {
            motorToggle = !motorToggle;
            if (motorToggle) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }
        });


        Button y_button = button(() -> gamepad1.y)
                .whenBecomesTrue(Turret.INSTANCE::resetRotateMotorPosition);

        Button dpad_up = button(() -> gamepad1.dpad_up)
                .whenBecomesTrue(Sort.INSTANCE.pushBallAndBack);
        Button dpad_down = button(() -> gamepad1.dpad_down)
                .whenBecomesTrue(Sort.INSTANCE.tripleLaunch);
        //   Button dpad_left = button(() -> gamepad1.dpad_left).whenBecomesTrue(()->{follower().setPose(new Pose(0,0,0));});
        // Button dpad_down = button(() -> gamepad1.dpad_down).whenBecomesTrue(Sort.INSTANCE.cycleRight.thenWait(.5).then(Sort.INSTANCE.cycleLeft));

        Button left_bumper = button(() -> gamepad1.left_bumper)
                .whenBecomesTrue(Sort.INSTANCE.cycleLeft);

        Button right_bumper = button(() -> gamepad1.right_bumper)
                .whenBecomesTrue(Sort.INSTANCE.cycleRight);

//        Button right_trigger = range(() -> gamepad1.right_trigger)
//                .greaterThan(0.2)
//                .whenBecomesTrue(Sort.INSTANCE.loadGreen)
//                .whenBecomesFalse(Sort.INSTANCE.shootGreen);

//        Button left_trigger = range(() -> gamepad1.left_trigger)
//                .greaterThan(0.2)
//                .whenBecomesTrue(Sort.INSTANCE.loadPurp)
//                .whenBecomesFalse(Sort.INSTANCE.shootPurp);

//        intake.setPower(1);

        follower().startTeleopDrive();

        DriverControlledCommand driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
//                range(() -> {
//                    double v = Gamepads.gamepad1().leftStickY().get();
//                    double exp = Math.copySign(Math.pow(Math.abs(v), 3.6), v);
//                    return -exp;
//                }),
//                range(() -> {
//                    double v = Gamepads.gamepad1().leftStickX().get();
//                    double exp = Math.copySign(Math.pow(Math.abs(v), 3.6), v);
//                    return -exp;
//                }),
                Gamepads.gamepad1().rightStickX().negate(),
                false
        );
        driverControlled.schedule();


    }

    @Override
    public void onUpdate() {
        if (!follower().getPose().roughlyEquals(antiCrazy.getLastMeasuredPose(), 15)) {
            /* this is my feeble attempt to convince the machine to NOT BREAK RANDOMLY
             * Basically, we check how close our pose is to the last one, if it's super different, as this
             * updates like a lot big fast, we should know that the pose had gone crazy and to ignore the last reading
             * maybe
             * hopefully */
            follower().setPose(antiCrazy.getLastMeasuredPose());
        } else {
            antiCrazy.updateLastPose(follower().getPose());
        }
        double turretPos = Turret.INSTANCE.getRotateMotorPosition();
        double turretWant = Turret.INSTANCE.getRotateMotorPosition() + Turret.INSTANCE.calculatePosition();
        //Turret.INSTANCE.setSetTurretVelocity(newVelocity);
        Turret.INSTANCE.setXoffset(xOffset);

        //Turret.INSTANCE.rebuildControlSystem(kp, ki, kd, kf,power);
        // Sort.INSTANCE.rebuildControlSystem(kp,ki,kd,kf,power);
        telemetryManager.addData("SpindexMotorPosition", Sort.INSTANCE.getCurrentPosition());
        if (Sort.INSTANCE.getSpinLimitSwitchStatus()) {
            telemetryManager.addData("FIRE READY", "");
        } else {
            telemetryManager.addData("FIRE NOT READY", "");
        }
        telemetryManager.addData("turretMotorPosition", turretPos);
        telemetryManager.addData("SpinNextPosition", Sort.INSTANCE.getTargetPosition());
        telemetryManager.addData("TurretNextPosition", turretWant);
        telemetryManager.addData("error", Math.abs(Sort.INSTANCE.getCurrentPosition() - Sort.INSTANCE.getTargetPosition()));
        telemetryManager.addData("Colors", Sort.INSTANCE.getColorArray());
        telemetryManager.addData("Pipeline", Turret.INSTANCE.limelightProcessing.getCurrentPipeline());
        telemetryManager.addData("Limelight Status", Turret.INSTANCE.limelightProcessing.limelightTelemetry());
        telemetryManager.addData("Alliance", Turret.INSTANCE.getSide());
        telemetryManager.addData("turretVelocity", Turret.INSTANCE.getTurretVelocity());
        // telemetryManager.addData("desiredVelocity",newVelocity);
        telemetryManager.update(telemetry);

    }


}
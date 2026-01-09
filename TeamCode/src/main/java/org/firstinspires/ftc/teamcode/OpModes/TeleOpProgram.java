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
import org.firstinspires.ftc.teamcode.Subsystems.MySubsystemGroup;
import org.firstinspires.ftc.teamcode.Subsystems.PTO;
import org.firstinspires.ftc.teamcode.Subsystems.Sort;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Sort.Color;

import java.util.Arrays;

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
    public static double newVelocity = 1345;
    //    public static double kp =0.004;
//    public static double ki = 0.004;
//    public static double kd =0.00026;
//    public static double kf = 0.0000275;
    //launcher
//    private static double kp =0.005;
//    private static double ki =  0.04;
//    private static double kd =  0.0;
//    private static double kf = 0.0004;

    private static double kp =0.002;
    private static double ki =  0.00;
   private static double kd =  0.00001;
    private static double ks =  0.000;

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


    public TeleOpProgram() {
        addComponents(
                new SubsystemComponent(Sort.INSTANCE),
                new SubsystemComponent(Turret.INSTANCE),
                new SubsystemComponent(PTO.INSTANCE),
                new SubsystemComponent(MySubsystemGroup.INSTANCE),

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

                });
                y_button.whenBecomesTrue(() -> {
                    RobotConfig.alliance = RobotConfig.Alliance.BLUE;
                    RobotConfig.autonomousStartEndPoses = RobotConfig.AutonomousStartEndPoses.GOALSIDEBLUE;
                    telemetryManager.addData("Start Selected:", " BLUE GOAL SIDE autonomous");
                    telemetryManager.update(telemetry);
                    sideSelected=true;

                });
                a_button.whenBecomesTrue(() -> {
                    RobotConfig.alliance = RobotConfig.Alliance.RED;
                    RobotConfig.autonomousStartEndPoses = RobotConfig.AutonomousStartEndPoses.FARSIDERED;
                    telemetryManager.addData("Start Selected:", " RED FAR SIDE autonomous");
                    telemetryManager.update(telemetry);
                    sideSelected=true;

                });
                b_button.whenBecomesTrue(() -> {
                    RobotConfig.alliance = RobotConfig.Alliance.RED;
                    RobotConfig.autonomousStartEndPoses = RobotConfig.AutonomousStartEndPoses.GOALSIDERED;
                    telemetryManager.addData("Start Selected:", " RED GOAL SIDE autonomous");
                    telemetryManager.update(telemetry);
                    sideSelected=true;

                });
            }
        }
        antiCrazy.updateLastPose(RobotConfig.finalMeasuredPose);
        if(sideSelected) {
            follower().setPose(startPose);
        }
        Turret.INSTANCE.resetRotateMotorPosition();
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Turret.INSTANCE.limelightProcessing.getLimelightStatus();
    }

    @Override
    public void onStartButtonPressed() {
        Button dpad_up = button(() -> gamepad1.dpad_up).whenBecomesTrue(() ->
        {
            motorToggle = !motorToggle;
            if (motorToggle) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }
        });

        Button dpad_down = button(() -> gamepad1.dpad_down).whenBecomesTrue(() ->
        {
            motorToggle = !motorToggle;
            if (motorToggle) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }
        });




        Button x_button = button(() -> gamepad1.x)
                .whenBecomesTrue(Sort.INSTANCE.shootGreen);
        Button y_button = button(() -> gamepad1.y)
                .whenBecomesTrue(Sort.INSTANCE.pushBallAndBack);
        Button a_button = button(() -> gamepad1.a)
                .whenBecomesTrue(MySubsystemGroup.INSTANCE.shootInPattern);
        Button b_button = button(() -> gamepad1.b)
                .whenBecomesTrue(Sort.INSTANCE.shootPurp);


        Button left_bumper = button(() -> gamepad1.left_bumper)
                .whenBecomesTrue(()->{
                   // Sort.INSTANCE.rebuildControlSystem(kp,ki,kd);
                    Sort.INSTANCE.cycleLeft.schedule();

                });

        Button right_bumper = button(() -> gamepad1.right_bumper)
                .whenBecomesTrue(Sort.INSTANCE.cycleRight);

        Button PTOEngage = button(() -> gamepad2.y)
                .whenBecomesTrue(()->{
                    follower().breakFollowing();
                    PTO.INSTANCE.engage.schedule();
                }
                );
        Button PTOLeftSide =  button(() -> gamepad2.left_bumper);
        Button PTORightSide =  button(() -> gamepad2.right_bumper);
        PTOLeftSide.whenTrue(()->{
            leftBack.setPower(1.0);
               PTO.INSTANCE.engageL.schedule(); }
            );
        PTOLeftSide.whenBecomesFalse(()->leftBack.setPower(0.0));
        PTORightSide.whenTrue(()->{rightBack.setPower(-1.0);
            PTO.INSTANCE.engageR.schedule(); }
        );
        PTORightSide.whenBecomesFalse(()->rightBack.setPower(0.0));


        follower().startTeleopDrive();

        DriverControlledCommand driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate(),
                false
        );
        driverControlled.schedule();


    }

    @Override
    public void onUpdate() {
        if(antiCrazy.getLastMeasuredPose()!=null) {
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
        }
        else if(RobotConfig.finalMeasuredPose!=null&&ActiveOpMode.isStarted()){
            antiCrazy.updateLastPose(RobotConfig.finalMeasuredPose);
        }

        double turretWant = Turret.INSTANCE.getRealTurretPosition() + Turret.INSTANCE.calculatePosition();
        //Turret.INSTANCE.setSetTurretVelocity(newVelocity);
        Turret.INSTANCE.setXoffset(xOffset);

        Sort.INSTANCE.getSpindexEncoder().updateRotations();
        telemetryManager.addData("SPINDEX ENCODER","");
        telemetryManager.addData("Raw Voltage", String.format("%.3f V", Sort.INSTANCE.getSpindexEncoder().getVoltage()));
        telemetryManager.addData("Max Voltage", String.format("%.3f V", Sort.INSTANCE.getSpindexEncoder().getMaxVoltage()));
        telemetryManager.addData("", "");
        telemetryManager.addData("Absolute Position", String.format("%.2f°", Sort.INSTANCE.getSpindexEncoder().getDegrees()));
        telemetryManager.addData("Position (Radians)", String.format("%.3f rad", Sort.INSTANCE.getSpindexEncoder().getRadians()));
        telemetryManager.addData("Percentage", String.format("%.1f%%", Sort.INSTANCE.getSpindexEncoder().getPercentage()));
        telemetryManager.addData("", "");
        telemetryManager.addData("=== ROTATION TRACKING ===", "");
        telemetryManager.addData("Rotations", Sort.INSTANCE.getSpindexEncoder().getRotations());
        telemetryManager.addData("Total Degrees", String.format("%.2f°", Sort.INSTANCE.getSpindexEncoder().getTotalDegrees()));
        telemetryManager.addData("Total Radians", String.format("%.3f rad", Sort.INSTANCE.getSpindexEncoder().getTotalRadians()));
        telemetryManager.addData("Goal",Sort.INSTANCE.getSpindexControl().getGoal().getPosition());

        Turret.INSTANCE.getRotateEncoder().updateRotations();
        telemetryManager.addData("ROTATE ENCODER","");
        telemetryManager.addData("Raw Voltage", String.format("%.3f V", Turret.INSTANCE.getRotateEncoder().getVoltage()));
        telemetryManager.addData("Max Voltage", String.format("%.3f V", Turret.INSTANCE.getRotateEncoder().getMaxVoltage()));
        telemetryManager.addData("", "");
        telemetryManager.addData("Absolute Position", String.format("%.2f°", Turret.INSTANCE.getRotateEncoder().getDegrees()));
        telemetryManager.addData("Position (Radians)", String.format("%.3f rad", Turret.INSTANCE.getRotateEncoder().getRadians()));
        telemetryManager.addData("Percentage", String.format("%.1f%%", Turret.INSTANCE.getRotateEncoder().getPercentage()));
        telemetryManager.addData("", "");
        telemetryManager.addData("=== ROTATION TRACKING ===", "");
        telemetryManager.addData("Rotations", Turret.INSTANCE.getRotateEncoder().getRotations());
        telemetryManager.addData("Total Degrees", String.format("%.2f°", Turret.INSTANCE.getRotateEncoder().getTotalDegrees()));
        telemetryManager.addData("Total Radians", String.format("%.3f rad", Turret.INSTANCE.getRotateEncoder().getTotalRadians()));

        telemetryManager.addData("goalvel",newVelocity);
        //Turret.INSTANCE.rebuildControlSystem(kp, ki, kd, kf,power);
        telemetryManager.addData("turretMotorPosition", Turret.INSTANCE.getRealTurretPosition());
        Sort.INSTANCE.rebuildControlSystem(kp, ki, kd,ks);
        telemetryManager.addData("TurretNextPosition", turretWant);
        if(Sort.INSTANCE.getColorArray()!=null) {
            Color[] colors = Sort.INSTANCE.getColorArray();
            telemetryManager.addData(
                    "Colors",
                    colors[0] + ", " + colors[1] + ", " + colors[2]
            );
        }
        telemetryManager.addData("Pipeline", Turret.INSTANCE.limelightProcessing.getCurrentPipeline());
        telemetryManager.addData("Limelight Status", Turret.INSTANCE.limelightProcessing.limelightTelemetry());
        telemetryManager.addData("turretVelocity", Turret.INSTANCE.getTurretVelocity());
        // telemetryManager.addData("desiredVelocity",newVelocity);
        telemetryManager.update(telemetry);

    }


}
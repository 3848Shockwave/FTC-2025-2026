package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.impl.MotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.Sort;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import static dev.nextftc.bindings.Bindings.button;
import static dev.nextftc.bindings.Bindings.range;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;//most important one to import


@Configurable
@TeleOp(name = "TeleOp Program", group = "Production")
public class TeleOpProgram extends NextFTCOpMode {
//    private final Pose startPose = new Pose(28.5, 128, Math.toRadians(180)); // Start Pose of our robot.

    MotorEx intake = new MotorEx("intake").brakeMode();




    public TeleOpProgram(){
        addComponents(
                new SubsystemComponent(Sort.INSTANCE),
                new SubsystemComponent(Turret.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
                );
    }


    private boolean motorToggle = false;
    private boolean launchToggle = false;

    private static double kp = 0.000;
    private static double kd = 0.000;
    private static double ki = 0.00;
    private static double kf = 0.0000;
    private static double power = 0.0;

    private TelemetryManager telemetryManager;

    @Override
    public void onInit() {
        Turret.INSTANCE.resetRotateMotorPosition();
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        Button a_button = button(() -> gamepad1.a).whenBecomesTrue(() ->
        {
            motorToggle = !motorToggle;
            if (motorToggle) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }
        });

        Button b_button = button(() -> gamepad1.b).whenBecomesTrue(()->
        {
            motorToggle = !motorToggle;
            if (motorToggle) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }
        });

        Button x_button = button(() -> gamepad1.x).whenBecomesTrue(()->
        {
            launchToggle = !launchToggle;
            if (launchToggle) {
                Turret.INSTANCE.setLaunchMotorSpeed(1.0);
            } else {
                Turret.INSTANCE.setLaunchMotorSpeed(0.0);
            }
        });
        Button y_button = button(() -> gamepad1.y)
                .whenBecomesTrue(Turret.INSTANCE::resetRotateMotorPosition);

        Button dpad_up = button(() -> gamepad1.dpad_up)
                .whenBecomesTrue(Sort.INSTANCE.pushBallAndBack);

        Button left_bumper = button(() -> gamepad1.left_bumper)
                .whenBecomesTrue(Sort.INSTANCE.cycleLeft);

        Button right_bumper = button(() -> gamepad1.right_bumper)
                .whenBecomesTrue(Sort.INSTANCE.cycleRight);

        Button right_trigger = range(() -> gamepad1.right_trigger)
                .greaterThan(0.2)
                .whenBecomesTrue(Sort.INSTANCE.loadGreen)
                .whenBecomesFalse(Sort.INSTANCE.shootGreen);

        Button left_trigger = range(() -> gamepad1.left_trigger)
                .greaterThan(0.2)
                .whenBecomesTrue(Sort.INSTANCE.loadPurp)
                .whenBecomesFalse(Sort.INSTANCE.shootPurp);

        Turret.INSTANCE.limelightProcessing.getLimelightStatus();
    }

    @Override
    public void onStartButtonPressed(){


//        intake.setPower(1);

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
        double turretPos = Turret.INSTANCE.getRotateMotorPosition();
        double turretWant = Turret.INSTANCE.getRotateMotorPosition()+Turret.INSTANCE.calculatePosition();

        //Turret.INSTANCE.rebuildControlSystem(kp, ki, kd, kf,power);
        Sort.INSTANCE.rebuildControlSystem(kp,ki,kd,kf,power);
        telemetryManager.addData("MotorPosition", Sort.INSTANCE.getCurrentPosition());
        telemetryManager.addData("NextPosition", Sort.INSTANCE.getTargetPosition() );
        telemetryManager.addData("error", Math.abs(Sort.INSTANCE.getCurrentPosition()-Sort.INSTANCE.getTargetPosition()) );
        telemetryManager.addData("Power",Sort.INSTANCE.getPowerToMove());
        telemetryManager.addData("Limelight Status", Turret.INSTANCE.limelightProcessing.limelightTelemetry());
        telemetryManager.addData("LimitSwitch Status", Sort.INSTANCE.getSpinLimitSwitchStatus());
        telemetryManager.update(telemetry);

    }

}
package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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
import dev.nextftc.hardware.impl.ServoEx;

import org.firstinspires.ftc.teamcode.Subsystems.Sort;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import static dev.nextftc.bindings.Bindings.button;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;//most important one to import


@Configurable
@TeleOp(name = "TeleOp Program", group = "Production")
public class TeleOpProgram extends NextFTCOpMode {
//    private final Pose startPose = new Pose(28.5, 128, Math.toRadians(180)); // Start Pose of our robot.

    MotorEx intake = new MotorEx("intake").brakeMode();


    public TeleOpProgram(){
        addComponents(
                new SubsystemComponent(Sort.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
                );
    }


    private boolean motorToggle = false;

    private static double kp = 0.0;
    private static double kd = 0.0;
    private static double ki = 0.0;
    private static double kf = 0.0;

    @Override
    public void onInit() {
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

//        Button x_button = button(() -> gamepad1.x).whenBecomesTrue(()->
//        {
//            double speed = Turret.INSTANCE.getSpeedNeeded();
//            motorToggle = !motorToggle;
//            if (motorToggle) {
//                intake.setPower(speed);
//            } else {
//                intake.setPower(0);
//            }
//        });

    }

    @Override
    public void onStartButtonPressed(){

        Gamepads.gamepad1().leftBumper().whenBecomesTrue(
                Sort.INSTANCE.pushBallAndBack
        );


        follower().startTeleopDrive();
        DriverControlledCommand driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX(),
                false
        );
        driverControlled.schedule();

    }

    @Override
    public void onUpdate() {
        Turret.INSTANCE.rebuildControlSystem(kp, ki, kd, kf);
        telemetry.update();

    }

}
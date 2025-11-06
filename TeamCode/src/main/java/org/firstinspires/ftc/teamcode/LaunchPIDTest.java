package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import static dev.nextftc.bindings.Bindings.*;


@Configurable
@TeleOp(name = "LaunchPIDTest", group = "Testing")
public class LaunchPIDTest extends NextFTCOpMode {
    {
        addComponents(/* vararg components */);

    }
    private final MotorEx motorEx = new MotorEx("LaunchMotor");
    private TelemetryManager telemetryManager;

    private ControlSystem controller;


    private boolean motorToggle = false;
    private static double power = 0.75;
    private static double velocity = 0.0;
    private static double kp = 0.001;
    private static double ki = 0.6;
    private static double kd = 0.0009;

    private static double ff = 0.00043;

    @Override public void onInit() {
        controller = dev.nextftc.control.ControlSystem.builder()
                .velPid(kp, ki, kd)
                .basicFF(ff)
                .build();

        controller.setGoal(new KineticState(0.0,0.0,0.0));

        motorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Button a_button = button(() -> gamepad1.a).whenBecomesTrue(() ->
        {
            motorToggle = !motorToggle;


        });

        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();



    }
    @Override public void onWaitForStart() { }
    @Override public void onStartButtonPressed() {
    }
    @Override public void onUpdate() {

        BindingManager.update();


        if (!motorToggle) {
            controller.setGoal(new KineticState(0.0,0.0,0.0));

        }
        else{
            controller.setGoal(new KineticState(0.0, velocity, 0.0));

        }


        power = controller.calculate(motorEx.getState());
        motorEx.setPower(power);
        telemetryManager.addData("MotorVelocity", motorEx.getVelocity());
        telemetryManager.addData("Desired Velocity", velocity);

        telemetryManager.update();
    }
    @Override public void onStop() {
        BindingManager.reset();
    }
}
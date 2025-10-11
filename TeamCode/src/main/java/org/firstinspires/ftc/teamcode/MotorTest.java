package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import static dev.nextftc.bindings.Bindings.*;

@TeleOp(name = "Motor Test", group = "Testing")
public class MotorTest extends NextFTCOpMode {
    {
        addComponents(/* vararg components */);

    }
    private final MotorEx motorEx = new MotorEx("LaunchMotor");

    private boolean motorToggle = false;

    @Override public void onInit() {
        motorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Button a_button = button(() -> gamepad1.a).whenBecomesTrue(() ->
        {
            motorToggle = !motorToggle;
            if (motorToggle) {
                motorEx.setPower(0.75);
            } else {
                motorEx.setPower(0);
            }
        });

        Button b_button = button(() -> gamepad1.b).whenBecomesTrue(()->
        {
            motorToggle = !motorToggle;
            if (motorToggle) {
                motorEx.setPower(1.0);
            } else {
                motorEx.setPower(0);
            }
        });
    }
    @Override public void onWaitForStart() { }
    @Override public void onStartButtonPressed() {
        //motorEx.setPower(0.75);
    }
    @Override public void onUpdate() {
        BindingManager.update();
    }
    @Override public void onStop() {
        BindingManager.reset();
    }
}
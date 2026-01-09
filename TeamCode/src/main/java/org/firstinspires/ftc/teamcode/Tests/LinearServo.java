package org.firstinspires.ftc.teamcode.Tests;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.impl.ServoEx;

@TeleOp(name = "Linear Servo Test", group = "Testing")
public class LinearServo extends NextFTCOpMode {

    private final ServoEx linearServoEx = new ServoEx("LinearServo");

    @Override public void onInit() {
        Button a_button = button(() -> gamepad1.a).whenBecomesTrue(() ->
                linearServoEx.setPosition(0.5));

        Button b_button = button(() -> gamepad1.b).whenBecomesTrue(() ->
                linearServoEx.setPosition(0));
    }
    @Override public void onWaitForStart() {}
    @Override public void onStartButtonPressed() {
        linearServoEx.setPosition(0.5);
    }
    @Override public void onUpdate() {
        BindingManager.update();
    }
    @Override public void onStop() {
        BindingManager.reset();
    }
}



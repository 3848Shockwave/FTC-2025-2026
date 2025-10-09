package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class OoriLearningToCode extends OpMode{
    @Override
    public void init() {

    }
    @Override
    public void loop() {
        //runs 50 times a second
        telemetry.addData("x", gamepad1.left_stick_x);
        telemetry.addData("y", gamepad1.left_stick_y);
        telemetry.addData("a", gamepad1.a);

    }

}


package org.firstinspires.ftc.teamcode.Tests;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.ServoEx;

@TeleOp(name = "Linear Servo Test", group = "Testing")
public class LinearServo extends NextFTCOpMode {
    Button x_button,y_button, a_button,b_button;
    public LinearServo(){
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    ServoEx linearServoEx = new ServoEx("liftServoLeft");

    ServoEx linearServoEx2 = new ServoEx("liftServoRight");


    @Override public void onInit() {
         a_button = button(() -> gamepad1.a).whenBecomesTrue(() ->{
             linearServoEx.to(1);
                 linearServoEx.setPosition(1);
                 telemetry.addData("Servo Position", linearServoEx.getPosition());
                     telemetry.update();
    }
        );

         b_button = button(() -> gamepad1.b).whenBecomesTrue(() ->{
                 telemetry.addData("Servo Position", linearServoEx.getPosition());
                 linearServoEx.setPosition(0);
             telemetry.update();}
        );

         x_button = button(() -> gamepad1.x).whenBecomesTrue(() ->
         {
             telemetry.addData("Servo Position", linearServoEx2.getPosition());
                linearServoEx2.setPosition(0);
             telemetry.update();}
        );
         y_button = button(() -> gamepad1.y).whenBecomesTrue(() ->{
             telemetry.addData("Servo Position", linearServoEx2.getPosition());
                linearServoEx2.setPosition(1);
             telemetry.update();}
        );
    }
    @Override public void onWaitForStart() {}
    @Override public void onStartButtonPressed() {
        a_button = button(() -> gamepad1.a).whenBecomesTrue(() ->{
                    linearServoEx.setPosition(1);
                    telemetry.addData("Servo Position", linearServoEx.getPosition());
                    telemetry.update();
                }
        );

        b_button = button(() -> gamepad1.b).whenBecomesTrue(() ->{
            telemetry.addData("Servo Position", linearServoEx.getPosition());
            linearServoEx.setPosition(0);
            telemetry.update();}
        );

        x_button = button(() -> gamepad1.x).whenBecomesTrue(() ->
                {
                    telemetry.addData("Servo Position", linearServoEx2.getPosition());
                    linearServoEx2.setPosition(0);
                    telemetry.update();}
        );
        y_button = button(() -> gamepad1.y).whenBecomesTrue(() ->{
            telemetry.addData("Servo Position", linearServoEx2.getPosition());
            linearServoEx2.setPosition(1);
            telemetry.update();}
        );
    }
    @Override public void onUpdate() {
        BindingManager.update();
    }
    @Override public void onStop() {
        BindingManager.reset();
    }
}



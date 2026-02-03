package org.firstinspires.ftc.teamcode.Tests;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.ServoEx;

@TeleOp(name="Spin Servo Position Test", group ="sensor")
public class SpinServoPositionMode extends NextFTCOpMode {

    public SpinServoPositionMode() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    private final ServoEx spindexRight = new ServoEx("spindexRight");
    private final ServoEx spindexLeft = new ServoEx("spindexLeft");


    public void spin (double index){

        spindexRight.to(index);
        spindexLeft.to(index);
    }

    public void toB (){
        spin(0.92);
    }

    public void toC(){
        spin(0.45);
    }

    public void toA(){
        spin(0);
    }



    @Override public void onInit() {
        Button x_button = button(() -> gamepad1.x)
                .whenBecomesTrue(()->toA());
        Button y_button = button(() -> gamepad1.y)
                .whenBecomesTrue(()->toC());

        Button a_button = button(() -> gamepad1.a)
                .whenBecomesTrue(()->toB());
    }

    @Override public void onStartButtonPressed() {

    }
    @Override public void onUpdate() {
        telemetry.addData("spinR", spindexRight.getPosition());
        telemetry.addData("spinL", spindexLeft.getPosition());
        telemetry.update();
    }
}

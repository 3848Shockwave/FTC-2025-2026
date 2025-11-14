package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.bindings.Range;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import static dev.nextftc.bindings.Bindings.*;


@Configurable
@TeleOp(name = "TriggerTest", group = "Testing")
public class TriggerTest extends NextFTCOpMode {
    {
        addComponents(/* vararg components */);

    }


    private TelemetryManager telemetryManager;
    private Range trigger_range;


    @Override public void onInit() {


        Button right_trigger = range(() -> gamepad1.right_trigger)
                .greaterThan(0.5)
                .whenBecomesTrue(() -> {
                    telemetryManager.addData("hello", "world");
                });

        // add value to print out range of trigger

        trigger_range = range(() -> gamepad1.right_trigger);



        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();


    }
    @Override public void onWaitForStart() { }
    @Override public void onStartButtonPressed() {
        //motorEx.setPower(0.75);
    }
    @Override public void onUpdate() {
        BindingManager.update();
        telemetryManager.addData("TriggerValue", trigger_range.get());
        telemetryManager.update(telemetry);

    }
    @Override public void onStop() {
        BindingManager.reset();
    }

    public void doSomething() {
        telemetryManager.addData("Hello", "World");

    }
}
package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;
import static dev.nextftc.bindings.Bindings.*;


@Configurable
@TeleOp(name = "ColorTest", group = "Testing")
public class ColorTest extends NextFTCOpMode {
    {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );

    }
    enum Color {
        NONE,
        GREEN,
        PURPLE
    }
    private TelemetryManager telemetryManager;

    ColorSensor colorSensor;



    @Override public void onInit() {

        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
    }
    @Override public void onWaitForStart() { }
    @Override public void onStartButtonPressed() {
        //motorEx.setPower(0.75);
        telemetryManager.addData("Red", colorSensor.red());
        telemetryManager.addData("Green", colorSensor.green());
        telemetryManager.addData("Blue", colorSensor.blue());
    }
    @Override public void onUpdate() {

        telemetryManager.addData("Red", colorSensor.red());
        telemetryManager.addData("Green", colorSensor.green());
        telemetryManager.addData("Blue", colorSensor.blue());

        telemetryManager.update();


    }

    @Override public void onStop() {
        BindingManager.reset();
    }
}
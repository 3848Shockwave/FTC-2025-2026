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

import androidx.lifecycle.viewmodel.CreationExtras;
//TODO finish checking l1/l2, r1/r2 using only print statements, no motors yet
//TODO create a "check color" function where driver can test if made oppsy
//TODO add spinning and shooting(final thing to do)

@Configurable
@TeleOp(name = "ColorTest", group = "Testing")
public class LearnToColor extends NextFTCOpMode {
    enum Color{
        GREEN, PURPLE, EMPTY;
    }
    private int tolerance = 0;
    private Color[] colorArray = {Color.EMPTY, Color.EMPTY, Color.EMPTY};
    {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );

    }

    private TelemetryManager telemetryManager;

    ColorSensor colorSensorL1;
    ColorSensor colorSensorL2;
    ColorSensor colorSensorR1;



    @Override public void onInit() {

        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        colorSensorL1 = hardwareMap.get(ColorSensor.class, "colorSensor");


    }
    @Override public void onWaitForStart() { }
    @Override public void onStartButtonPressed() {
        //motorEx.setPower(0.75);
       // NormalizedRGBA color = colorSensor.getNormalizedColors();
        telemetryManager.addData("Red", colorSensorL1.red());
        telemetryManager.addData("Green", colorSensorL1.green());
        telemetryManager.addData("Blue", colorSensorL1.blue());
    }
    @Override public void onUpdate() {
        //NormalizedRGBA color = colorSensor.getNormalizedColors();
        telemetryManager.addData("Red", colorSensorL1.red());
        telemetryManager.addData("Green", colorSensorL1.green());
        telemetryManager.addData("Blue", colorSensorL1.blue());

        telemetryManager.update();

        //check for green, need to add and statement for minus tolerance
          if (colorSensorL1.green() >= 4800 + tolerance || colorSensorL1.green() >= 4800 - tolerance ){

                colorArray[0] = Color.GREEN;
            }
          // else if L2

        //check for purple
            if ((colorSensorL1.green() < 4800 + tolerance || colorSensorL1.green() < Math.abs(4800 - tolerance)) && (colorSensorL1.green() > 2600 + tolerance || colorSensorL1.green() > Math.abs(2600 - tolerance))){
                colorArray[0] = Color.GREEN;
            }
            //else if R2

        
    }

    //@Override public void on (shoot purple?) (Left trigger?)

    //@override publice void on (shoot green?) (Right Trigger?)

    //@override public void on (check color) ("A" button pressed)

    @Override public void onStop() {
        BindingManager.reset();
    }
}
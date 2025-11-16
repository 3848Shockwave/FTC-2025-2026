package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OpModes.TeleOpProgram;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPositions;

public class Sort implements Subsystem {
    public static final Sort INSTANCE = new Sort();
    HardwareMap hw = ActiveOpMode.hardwareMap();
    Telemetry telemetry = ActiveOpMode.telemetry();
    DigitalChannel limitSwitch = hw.get(DigitalChannel.class, "spin");

    private Sort() { }
   // private final MotorEx turningPlate = new MotorEx("turningPlate").brakeMode();



    // 5203-2402-0051, 50.9:1, 8mm(D) shaft, 117rpm, 1,425.1 PPR, 1:2 gear ratio. --> 72637.59 impulses per rotation
    //however, with the gear structure, the palate make a full turn every 145075.18 impulses, since there are 3 circle
    //every position should take exactly 48358 pulse


    //private final ServoEx servoLeft = new ServoEx("servoLeft");
   // private final ServoEx servoRight = new ServoEx("servoRight");

    NormalizedColorSensor colorSensorLeft1;
    NormalizedColorSensor colorSensorLeft2;
    NormalizedColorSensor colorSensorRight1;
    NormalizedColorSensor colorSensorRight2;
    private ControlSystem controlSystem;


    private TelemetryManager telemetryManager;



   // public final Command pushBall = new SetPositions(servoLeft.to(1), servoRight.to(1)).requires(this);
   // public final Command backPosition = new SetPositions(servoLeft.to(0), servoRight.to(0)).requires(this);


   // public final Command pushBallAndBack = pushBall.then(backPosition);
   // public final Command nextBall = new RunToPosition(controlSystem, 48358).requires(this).named("nextBall");

/*
    private String detectLeftColor() {
        String color = detectColor(colorSensorLeft1);
        if ("UNKNOWN".equals(color)) {
            color = detectColor(colorSensorLeft2);
        }
        return color;
    }

 */

    public boolean getSpinLimitSwitchStatus() {
        boolean isPressed =limitSwitch.getState(); // Assuming active low
        return isPressed;
    }
/*
    // Method to detect color with fallback for right group
    private String detectRightColor() {
        String color = detectColor(colorSensorRight1);
        if ("UNKNOWN".equals(color)) {
            color = detectColor(colorSensorRight2);
        }
        return color;
    }

    // Enhanced color detection method
    private String detectColor(NormalizedColorSensor sensor) {
        NormalizedRGBA colors = sensor.getNormalizedColors();

        // Add threshold to avoid false readings
        double threshold = 0.05; // Minimum intensity to consider a color

        if (colors.red < threshold && colors.green < threshold && colors.blue < threshold) {
            return "UNKNOWN"; // No significant color detected
        }

        if (colors.red > colors.blue && colors.red > colors.green) {
            return "RED";
        } else if (colors.blue > colors.red && colors.blue > colors.green) {
            return "BLUE";
        } else if (colors.green > colors.red && colors.green > colors.blue) {
            return "GREEN";
        } else {
            return "UNKNOWN";
        }
    }
*/









    @Override
    public void initialize() {

        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        limitSwitch.setState(false);
        // initialization logic (runs on init)
//        controlSystem = controlSystem.builder()
//                .posPid(0.01,0.6,0.009)
//                .basicFF(0.0005)
//                .build();


        //telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
//        colorSensorLeft1 = hardwareMap.get(NormalizedColorSensor.class, "colorSensorLeft1");
//        colorSensorLeft2 = hardwareMap.get(NormalizedColorSensor.class, "colorSensorLeft2");
//        colorSensorRight1 = hardwareMap.get(NormalizedColorSensor.class, "colorSensorRight1");
//        colorSensorRight2 = hardwareMap.get(NormalizedColorSensor.class, "colorSensorRight2");

    }

    @Override
    public void periodic() {
//        String leftColor = detectLeftColor();
//        String rightColor = detectRightColor();
//
//        telemetryManager.addData("Left Color", leftColor);
//        telemetryManager.addData("Right Color", rightColor);
//
//        // Keep the original telemetry for debugging
//        NormalizedRGBA color = colorSensorLeft2.getNormalizedColors();
//        telemetryManager.addData("Red", color.red);
//        telemetryManager.addData("Green", color.green);
//        telemetryManager.addData("Blue", color.blue);
    }

}

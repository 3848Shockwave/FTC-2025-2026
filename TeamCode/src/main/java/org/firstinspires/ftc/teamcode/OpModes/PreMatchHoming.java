package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.impl.MotorEx;


@TeleOp(name = "Pre-Match Homing", group = "Production")

public class PreMatchHoming extends NextFTCOpMode {
    //This class is used to home the turret before the match starts
    private final MotorEx rotateMotor = new MotorEx("rotateMotor").brakeMode();
    private final DigitalChannelImpl homeSwitch = hardwareMap.get(DigitalChannelImpl.class, "turretHomeSwitch");
    Telemetry telemetry;

    public void onStartButtonPressed() {
        rotateMotor.setPower(-.2);
        if (!homeSwitch.getState()) {
            rotateMotor.setPower(0);
            rotateMotor.setCurrentPosition(0);
            telemetry.addData("Homing Status: ", "Homing Complete");
            telemetry.update();
        }
        stop();
    }

}

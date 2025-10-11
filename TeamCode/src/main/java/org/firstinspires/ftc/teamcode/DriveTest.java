package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.driving.DriverControlledCommand;

// at the top of the file:
import static dev.nextftc.extensions.pedro.PedroComponent.follower;



@TeleOp(name = "DriveTest", group = "Testing")
public class DriveTest extends NextFTCOpMode {
    {
        addComponents(
                new PedroComponent(Constants::createFollower)
        );

    }








    @Override public void onInit() {
    }
    @Override public void onWaitForStart() { }
    @Override public void onStartButtonPressed() {
        DriverControlledCommand driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX(),
                false
        );
        driverControlled.schedule();


    }
    @Override public void onUpdate() {
        BindingManager.update();
    }
    @Override public void onStop() {
        BindingManager.reset();
    }
}

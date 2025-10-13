package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;

// at the top of the file:
import static dev.nextftc.extensions.pedro.PedroComponent.follower;
import com.pedropathing.geometry.Pose;



@TeleOp(name = "DriveTest", group = "Testing")
public class DriveTest extends NextFTCOpMode {
    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );

    }

    private final Pose startPose = new Pose(28.5, 128, Math.toRadians(180)); // Start Pose of our robot.
    // TODO: calibrate and check if this works first


    @Override public void onInit() {
        follower().setStartingPose(startPose);
        follower().update();
    }
    @Override public void onWaitForStart() {

    }
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

    }
    @Override public void onStop() {

    }
}

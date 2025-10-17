package org.firstinspires.ftc.teamcode;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;


@Configurable
@TeleOp(name = "AutoTest", group = "Testing")
public class AutoTest extends NextFTCOpMode {
    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );

    }

    public static Pose startPose = new Pose(0, 0, 0); // Start Pose of our robot.
    // TODO: calibrate and check if this works first

    private TelemetryManager telemetryManager;


    @Override
    public void onInit() {
        follower().setStartingPose(startPose); // this is how pedropathing does it normally i guess
        follower().update();

        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();

    }

    @Override
    public void onWaitForStart() {

    }

    @Override
    public void onStartButtonPressed() {



    }

    @Override
    public void onUpdate() {
        telemetryManager.update();

    }

    @Override
    public void onStop() {

    }
}

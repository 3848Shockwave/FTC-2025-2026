package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "BeamBreakerTest", group = "Testing")
public class BeamBreakerTest extends NextFTCOpMode {
    private DigitalChannel beamBreaker;

    @Override
    public void onInit(){
        beamBreaker = hardwareMap.get(DigitalChannel.class, "beamBreaker");
        beamBreaker.setMode(DigitalChannel.Mode.INPUT);
    }
    @Override
    public void onUpdate() {
        boolean isBeamBroken = beamBreaker.getState();

        telemetry.addData("Beam Broken", isBeamBroken);
        telemetry.update();
    }
}


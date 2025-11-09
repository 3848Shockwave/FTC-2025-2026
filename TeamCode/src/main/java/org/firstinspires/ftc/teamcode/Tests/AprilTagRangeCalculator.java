package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Helpers.LimelightProcessing;

import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "AprilTagRangeCalculator", group = "Testing")
public class AprilTagRangeCalculator extends NextFTCOpMode {
    // Calculator for AprilTag distance based on area readings
    // brogan pratt video needed to calculate scale factor: https://www.youtube.com/watch?v=Ap1lBywv00M
    int pipeline = 0;// Set the desired pipeline here
    boolean recording = false;
    double averageArea = 0;
    double minArea = 0;
    double maxArea = 0;
    LimelightProcessing limelightProcessing = new LimelightProcessing();
    TelemetryManager telemetryManager;

    @Override
    public void onInit() {

        limelightProcessing.initLimelight(pipeline);
        telemetryManager.addData("Status", "Initialized Limelight");
    }

    @Override
    public void onStartButtonPressed() {
        if (gamepad1.aWasPressed()) {
            recording = !recording;
        }
    }

    @Override
    public void onUpdate() {
        if (recording) {
            limelightProcessing.processTargets();
            double currentArea = limelightProcessing.getTargetInfo().getArea();
            if (currentArea > maxArea) {
                maxArea = currentArea;
            }
            if (currentArea < minArea) {
                minArea = currentArea;
            }
            averageArea = (averageArea + currentArea) / 2;
            telemetryManager.addData("Current Area", currentArea);
            telemetryManager.addData("Average Area", averageArea);
            telemetryManager.addData("Min Area", minArea);
            telemetryManager.addData("Max Area", maxArea);
            telemetryManager.update(telemetry);
        }
    }
}

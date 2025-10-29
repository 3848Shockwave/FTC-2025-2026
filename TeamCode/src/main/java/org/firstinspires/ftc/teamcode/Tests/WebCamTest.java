package org.firstinspires.ftc.teamcode.Tests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous
public class WebCamTest extends NextFTCOpMode {

    AprilTagWebCam aprilTagWebCam = new AprilTagWebCam();

    @Override
    public void onInit(){
        aprilTagWebCam.onInit(hardwareMap, telemetry);
    }
    @Override
    public void onUpdate() {
        aprilTagWebCam.onUpdate();
        AprilTagDetection id21 = aprilTagWebCam.getTagBySpecificID(21);
        aprilTagWebCam.displayDetectionTelemetry(id21);
        telemetry.addData("id21 String", id21 != null ? id21.toString() : "No tag detected");

    }

}
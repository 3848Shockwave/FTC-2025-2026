package org.firstinspires.ftc.teamcode.Tests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import dev.nextftc.ftc.NextFTCOpMode;


/*
need to test:
    1. whether the angle of april tag will affect the x value
    2. whether the data is absolute value or it will go negative
 */
@Autonomous
public class WebCamTest extends NextFTCOpMode {

    AprilTagWebCam aprilTagWebCam = new AprilTagWebCam();

    @Override
    public void onInit(){
        aprilTagWebCam.onInit(hardwareMap, telemetry);
        telemetry.update();
    }


    @Override
    public void onUpdate() {
        aprilTagWebCam.onUpdate();
        AprilTagDetection id21 = aprilTagWebCam.getTagBySpecificID(21);
        aprilTagWebCam.displayDetectionTelemetry(id21);
        telemetry.addData("id21 String", id21 != null ? id21.toString() : "No tag detected");
        telemetry.update();
    }

    /*
    XYZ: means the distance from camera to the object

    PRY: stands for pitch, roll, and yaw

    RBE: is range(center of your camera to center of your tag), bearing(angle of deflection away from the object),
    and elevation(how far up you are pasted off that)
     */

}
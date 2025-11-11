package org.firstinspires.ftc.teamcode.Subsystems.Helpers;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;

public class LimelightProcessing {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    ArrayList<TargetInfo> targetsDetected = new ArrayList<>();
    int currentPipeline = 0;
    private Limelight3A limelight;

    //Pipeline 0: AprilTag ID 21
    public void initLimelight(int pipeline) {
        hardwareMap = ActiveOpMode.hardwareMap();
       // limelight.updateRobotOrientation(); //This may be iffy, It needs to ge the yaw of the robot to do pose localization
        telemetry = ActiveOpMode.telemetry();
        targetsDetected = new ArrayList<>();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(pipeline);
        limelight.start();


    }

    public void getLimelightStatus() {
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            telemetry.addData("Limelight Status: ", "Targets Detected");
            telemetry.addData("current Pipeline: ", getCurrentPipeline());
            telemetry.update();
        } else {
            telemetry.addData("Limelight Status: ", "No Targets Detected");
            telemetry.update();
        }
    }

    public void setPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
        currentPipeline = pipeline;
    }

    public int getCurrentPipeline() {
        return currentPipeline;
    }

    public ArrayList<TargetInfo> processTargets() {
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            ArrayList<Double> detectedIDs = new ArrayList<>();

            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                double id = fr.getFiducialId();
                double x = fr.getTargetXDegrees();
                double y = fr.getTargetYDegrees();
                double area = fr.getTargetArea();
                detectedIDs.add(id);

                boolean updated = false;
                for (TargetInfo t : targetsDetected) {
                    if (t.getID() == id) {
                        // update existing target using setter methods
                        t.setX(x);
                        t.setY(y);
                        t.setA(area);
                        updated = true;
                        break;
                    }
                }

                if (!updated) {
                    targetsDetected.add(new TargetInfo(id, x, y, area));
                }
            }

        } else {
            targetsDetected.clear(); // Clear all targets if no valid results
        }
        return targetsDetected;
    }

    public TargetInfo getTargetInfo(double targetID) {
        for (TargetInfo t : processTargets()) {
            if (t.getID() == targetID) {
                return t;
            }
        }
        return null; // Return null if no target with the specified ID is found
    }

    public TargetInfo getTargetInfo() {
        if (!targetsDetected.isEmpty()) {
            return targetsDetected.get(0); // Return the first target detected
        }
        return null; // Return null if no targets are detected
    }
    public Pose3D getPose(){
        Pose3D pose;
        limelight.updateRobotOrientation(PedroComponent.follower().getHeading());
       LLResult result = limelight.getLatestResult();
       if (!result.isValid()) {
        pose = null;
       }
       pose = limelight.getLatestResult().getBotpose_MT2();
        return pose;
    }

    public void stopLimelight() {
        limelight.stop();
    }

    public String limelightTelemetry() {
        LLResult result = limelight.getLatestResult();
        StringBuilder output = new StringBuilder();

        if (result != null && result.isValid()) {
            output.append("Limelight Status: Targets Detected\n");
            output.append("Number of Targets: ").append(result.getFiducialResults().size()).append("\n");

            for (TargetInfo t : processTargets()) {
                for (java.lang.reflect.Field f : t.getClass().getDeclaredFields()) {
                    f.setAccessible(true);
                    try {
                        Object val = f.get(t);
                        output.append(f.getName()).append(": ").append(val).append("\n");
                    } catch (IllegalAccessException e) {
                        output.append(f.getName()).append(": access error").append("\n");
                    }
                }
                output.append("---\n");
            }
        } else {
            output.append("Limelight Status: No Targets Detected\n");
        }

        return output.toString();
    }
}


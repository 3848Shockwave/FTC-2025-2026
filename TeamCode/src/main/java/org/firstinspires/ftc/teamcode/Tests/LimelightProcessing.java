package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

import dev.nextftc.ftc.ActiveOpMode;

public class LimelightProcessing {
    private Limelight3A limelight;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    ArrayList<TargetInfo> targetsDetected = new ArrayList<>();

    public void initLimelight() {
        hardwareMap = ActiveOpMode.hardwareMap();
        telemetry = ActiveOpMode.telemetry();
        targetsDetected = new ArrayList<>();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();


    }
    public void getLimelightStatus(){
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            telemetry.addData("Limelight Status: ", "Targets Detected");
            telemetry.addData("Number of Targets: ", result.getFiducialResults().size());
            telemetry.update();
        }
        else{
            telemetry.addData("Limelight Status: ", "No Targets Detected");
            telemetry.update();
        }
    }
    public ArrayList<TargetInfo> processTargets(){
        LLResult result = limelight.getLatestResult();

        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                double id = fr.getFiducialId();
                double x = fr.getTargetXDegrees();
                double y = fr.getTargetYDegrees();
                double area = fr.getTargetArea();

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

        }
        else{
            telemetry.addData("Limelight Status: ", "No Targets Detected");
            telemetry.update();
        }
        return targetsDetected;
    }

    public TargetInfo getTargetInfoByID(double targetID){
        for (TargetInfo t : processTargets()) {
            if (t.getID() == targetID) {
                return t;
            }
        }
        return null; // Return null if no target with the specified ID is found
    }
    public void stopLimelight(){
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
                               output.append(f.getName()).append(": ").append(String.valueOf(val)).append("\n");
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


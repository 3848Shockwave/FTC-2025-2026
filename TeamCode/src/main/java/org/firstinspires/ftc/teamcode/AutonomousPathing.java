package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;


import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
public class AutonomousPathing extends LinearOpMode {
    
    private DriveSubsystem drive;
    
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the drive subsystem
        drive = new DriveSubsystem(hardwareMap);
        
        // Reset the robot's position
        drive.getFollower().setStartingPose(new Pose(0, 0, 0));
        
        // Create paths for the autonomous using BezierLine
        Path forwardPath = new Path(new BezierLine(new Pose(0, 0, 0), new Pose(0, 24, 0)));
        forwardPath.setConstantHeadingInterpolation(0);
        
        Path strafePath = new Path(new BezierLine(new Pose(0, 24, 0), new Pose(24, 24, 0)));
        strafePath.setConstantHeadingInterpolation(0);
        
        Path returnPath = new Path(new BezierLine(new Pose(24, 24, 0), new Pose(0, 0, 0)));
        returnPath.setConstantHeadingInterpolation(0);
        
        // Create a path chain
        PathChain pathChain = drive.getFollower().pathBuilder()
                .addPath(forwardPath)
                .addPath(strafePath)
                .addPath(returnPath)
                .build();
        
        // Wait for the start button to be pressed
        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();
        
        // Start following the path chain
        drive.getFollower().followPath(pathChain);
        
        // Main loop - update the follower until the path is complete
        while (opModeIsActive() && drive.getFollower().isBusy()) {
            drive.getFollower().update();
            
            // Display telemetry
            Pose currentPose = drive.getFollower().getPose();
            telemetry.addData("X", currentPose.getX());
            telemetry.addData("Y", currentPose.getY());
            telemetry.addData("Heading", Math.toDegrees(currentPose.getHeading()));
            telemetry.update();
            
            sleep(10);
        }
        
        // Stop the robot
        drive.driveFieldCentric(0, 0, 0);
        
        // Final telemetry
        telemetry.addData("Status", "Autonomous Complete");
        telemetry.update();
    }
}
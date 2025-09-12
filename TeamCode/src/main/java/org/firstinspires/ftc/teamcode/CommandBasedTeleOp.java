package org.firstinspires.ftc.teamcode;

// FTCLib imports - the command-based framework
import com.arcrobotics.ftclib.command.CommandOpMode;    // Base class for FTCLib command-based OpModes
import com.arcrobotics.ftclib.command.RunCommand;       // Simple command that runs a method repeatedly
import com.arcrobotics.ftclib.command.button.GamepadButton; // Wrapper for gamepad buttons with trigger events
import com.arcrobotics.ftclib.gamepad.GamepadEx;        // Enhanced gamepad with better input handling
import com.arcrobotics.ftclib.gamepad.GamepadKeys;      // Enum for gamepad button constants
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;  // Standard FTC TeleOp annotation

@TeleOp(name = "Command Based TeleOp", group = "Command Based")
public class CommandBasedTeleOp extends CommandOpMode { // Extends CommandOpMode instead of OpMode for FTCLib
    
    // VARIABLES:
    private DriveSubsystem drive;    // Our custom subsystem that controls the 4 mecanum wheels
    private GamepadEx gamepadEx;     // Enhanced gamepad wrapper with better input methods than gamepad1
    private PanelsSubsystem panels;  // Panels telemetry and visualization subsystem

    @Override
    public void initialize() {  // FTCLib uses initialize() instead of init() - called once when OpMode starts
        
        // Initialize our drive subsystem with hardware map
        drive = new DriveSubsystem(hardwareMap); // Creates subsystem and sets up motors + Pinpoint sensor
        
        // Initialize Panels subsystem
        panels = new PanelsSubsystem();
        
        // Wrap the standard gamepad1 in FTCLib's enhanced GamepadEx
        gamepadEx = new GamepadEx(gamepad1);     // Provides better input handling and button triggering
        
        // SET DEFAULT COMMAND - this runs continuously when no other command is using the drive subsystem
        drive.setDefaultCommand(new RunCommand(  // RunCommand executes a lambda function repeatedly
                () -> {
                    // Get current robot position and heading
                    double x = drive.getFollower().getPose().getX();
                    double y = drive.getFollower().getPose().getY();
                    double heading = drive.getFollower().getPose().getHeading();
                    
                    // Drive the robot
                    drive.driveFieldCentric(
                            -gamepadEx.getLeftY(),   // Forward/backward: negative because Y-axis is inverted on gamepads
                            gamepadEx.getLeftX(),    // Left/right strafe: positive right
                            gamepadEx.getRightX()    // Rotation: positive clockwise
                    );
                    
                    // Update Panels telemetry
                    panels.updateTelemetry(x, y, heading, "Driving", "Use joysticks to move");
                    
                    // Draw robot on field
                    panels.drawRobot(x, y, heading);
                    
                    // Send updates to Panels
                    panels.updateField();
                }, 
                drive                         // Second parameter: tells FTCLib this command requires the drive subsystem
        ));
        
        // BUTTON BINDING - creates a button trigger that executes code when pressed
        new GamepadButton(gamepadEx, GamepadKeys.Button.A)  // Creates button wrapper for A button
                .whenPressed(() -> {
                    drive.resetHeading();   // Reset field orientation to 0°
                    panels.getTelemetryManager().debug("Heading Reset");
                    panels.getTelemetryManager().update();
                });
        
        // Display initialization info to driver station
        telemetry.addData("Status", "FTCLib Command Based Ready");
        telemetry.addData("Controls", "Left stick: drive, Right stick: turn, A: reset heading");
    }
    
    // NOTE: No loop() method needed! FTCLib CommandOpMode handles the command scheduling automatically
    // - The default RunCommand continuously calls driveFieldCentric()
    // - Button triggers are monitored automatically
    // - Subsystem periodic() methods are called automatically
}
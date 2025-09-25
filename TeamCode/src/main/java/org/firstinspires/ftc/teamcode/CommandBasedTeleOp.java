package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandBase.DriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.PanelsSubsystem;

@TeleOp(name = "Command Based TeleOp", group = "Command Based")
public class CommandBasedTeleOp extends CommandOpMode {
    
    // VARIABLES:
    private DriveSubsystem drive;
    private GamepadEx gamepadEx;
    private PanelsSubsystem panels;

    @Override
    public void initialize() {
        
        // Initialize our drive subsystem with hardware map
        drive = new DriveSubsystem(hardwareMap);

        // Initialize Panels subsystem
        panels = new PanelsSubsystem();

        // Wrap the standard gamepad1 in FTCLib's enhanced GamepadEx
        gamepadEx = new GamepadEx(gamepad1);

        // SET DEFAULT COMMAND - this runs continuously when no other command is using the drive subsystem
        drive.setDefaultCommand(new DriveCommand(drive, gamepadEx));
        
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
    

}
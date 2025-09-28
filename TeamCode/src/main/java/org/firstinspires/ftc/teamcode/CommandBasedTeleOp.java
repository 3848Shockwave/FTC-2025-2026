package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandBase.DriveCommand;
import org.firstinspires.ftc.teamcode.CommandBase.ArmManualControlCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.PanelsSubsystem;

@TeleOp(name = "Command Based TeleOp", group = "Command Based")
public class CommandBasedTeleOp extends CommandOpMode {
    
    // VARIABLES:
    private DriveSubsystem drive;
    private ArmSubsystem arm;
    private GamepadEx driverGamepad;
    private PanelsSubsystem panels;

    @Override
    public void initialize() {
        

        drive = new DriveSubsystem(hardwareMap);

        arm = new ArmSubsystem(hardwareMap);

        panels = new PanelsSubsystem();

        driverGamepad = new GamepadEx(gamepad1);

        // SET DEFAULT COMMANDS - these run continuously when no other command is using the subsystems
        drive.setDefaultCommand(new DriveCommand(drive, driverGamepad));
        arm.setDefaultCommand(new ArmManualControlCommand(arm, driverGamepad));
        



        
        // Display initialization info to driver station
        telemetry.addData("Status", "FTCLib Command Based Ready");
        telemetry.addData("Driver Controls", "Left stick: drive, Right stick: turn, Right trigger: lift up, Left trigger: lift down");
        telemetry.addData("Other Controls", "Right stick X: arm extend, A: reset heading, B: stop arm");
        telemetry.update();
    }
}
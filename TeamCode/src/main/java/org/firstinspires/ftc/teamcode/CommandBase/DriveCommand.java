package org.firstinspires.ftc.teamcode.CommandBase;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;


public class DriveCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final GamepadEx gamepad;


    public DriveCommand(DriveSubsystem driveSubsystem, GamepadEx gamepad) {
        this.driveSubsystem = driveSubsystem;
        this.gamepad = gamepad;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {

        driveSubsystem.driveFieldCentric(
                -gamepad.getLeftY(),
                gamepad.getLeftX(),
                gamepad.getRightX()
        );
    }

    @Override
    public boolean isFinished() {

        return false;
    }


    @Override
    public void end(boolean interrupted) {

        if (interrupted) {
            driveSubsystem.driveFieldCentric(0, 0, 0);
        }
    }
}
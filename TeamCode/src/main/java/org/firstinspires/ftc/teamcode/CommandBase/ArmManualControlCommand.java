package org.firstinspires.ftc.teamcode.CommandBase;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;


public class ArmManualControlCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final GamepadEx gamepad;

    public ArmManualControlCommand(ArmSubsystem armSubsystem, GamepadEx gamepad) {
        this.armSubsystem = armSubsystem;
        this.gamepad = gamepad;
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        double upperPower = gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER) ? 1 : -1;
        double lowerPower = gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER) ? 1 : -1;

        double liftPower = upperPower - lowerPower;
        armSubsystem.liftArm(liftPower);
        

        armSubsystem.extendArm(gamepad.getRightX());
    }

    @Override
    public boolean isFinished() {

        return false;
    }


    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            armSubsystem.stop();
        }
    }
}
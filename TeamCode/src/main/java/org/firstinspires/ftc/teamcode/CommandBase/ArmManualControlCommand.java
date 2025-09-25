package org.firstinspires.ftc.teamcode.CommandBase;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

/**
 * ArmManualControlCommand - 手动控制机械臂的命令类
 * 
 * 这个命令允许操作员通过游戏手柄控制机械臂的升降和伸缩
 */
public class ArmManualControlCommand extends CommandBase {
    
    // 命令需要的组件
    private final ArmSubsystem armSubsystem;
    private final GamepadEx gamepad;

    /**
     * 构造函数 - 初始化命令所需的所有组件
     * @param armSubsystem 机械臂子系统
     * @param gamepad 游戏手柄
     */
    public ArmManualControlCommand(ArmSubsystem armSubsystem, GamepadEx gamepad) {
        this.armSubsystem = armSubsystem;
        this.gamepad = gamepad;
        
        // 声明此命令需要使用armSubsystem资源
        addRequirements(armSubsystem);
    }

    /**
     * execute() - 每个循环执行一次
     * 这是命令的主要逻辑
     */
    @Override
    public void execute() {
        // 控制机械臂升降 (使用右摇杆的Y轴)
        armSubsystem.liftArm(-gamepad.getRightY());
        
        // 控制机械臂伸缩 (使用右摇杆的X轴)
        armSubsystem.extendArm(gamepad.getRightX());
    }

    /**
     * isFinished() - 决定命令何时完成
     * @return true表示命令完成，false表示继续执行
     * 
     * 对于手动控制命令，我们希望它一直运行，直到被其他命令中断
     * 所以总是返回false
     */
    @Override
    public boolean isFinished() {
        // 手动控制命令是一个持续性命令，永远不会自动完成
        // 它只会在被更高优先级的命令中断时才会结束
        return false;
    }

    /**
     * end() - 当命令结束时调用（无论是正常结束还是被中断）
     * @param interrupted true表示命令被中断，false表示正常结束
     */
    @Override
    public void end(boolean interrupted) {
        // 当命令结束时，确保机械臂停止
        // 这在命令被中断时特别重要
        if (interrupted) {
            armSubsystem.stop();
        }
    }
}
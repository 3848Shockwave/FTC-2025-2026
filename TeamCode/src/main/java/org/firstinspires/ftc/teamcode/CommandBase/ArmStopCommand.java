package org.firstinspires.ftc.teamcode.CommandBase;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;


public class ArmStopCommand extends CommandBase {
    
    // 命令需要的组件
    private final ArmSubsystem armSubsystem;


    public ArmStopCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        
        // 声明此命令需要使用armSubsystem资源
        addRequirements(armSubsystem);
    }

    /**
     * initialize() - 命令开始时执行一次
     * 这是命令的主要逻辑
     */
    @Override
    public void initialize() {
        // 停止机械臂
        armSubsystem.stop();
    }

    /**
     * isFinished() - 决定命令何时完成
     * @return true表示命令完成，false表示继续执行
     * 
     * 对于停止命令，执行一次就完成
     */
    @Override
    public boolean isFinished() {
        // 停止命令执行一次就完成
        return true;
    }
}
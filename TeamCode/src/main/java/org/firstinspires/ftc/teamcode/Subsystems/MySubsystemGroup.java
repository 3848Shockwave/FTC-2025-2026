package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.Timer;

import dev.nextftc.core.commands.Command;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.SubsystemGroup;

public class MySubsystemGroup extends SubsystemGroup {
    public static final MySubsystemGroup INSTANCE = new MySubsystemGroup();
    private boolean checkIsDone =false;
    private int counter =0;
    private static ElapsedTime timer = new ElapsedTime();
    private Sort.Color[] colorWeHave = new Sort.Color[3];
    private Sort.Color[] targetColor = new Sort.Color[3];



    private MySubsystemGroup() {
        super(
                Turret.INSTANCE,
                Sort.INSTANCE
        );
    }

    public boolean containsColor (Sort.Color color) {
        for (Sort.Color c : colorWeHave) {
            if (c == color) {
                return true;
            }
        }
        return false;
    }

    public Command shootInPattern = new InstantCommand(() -> {
            if(Arrays.equals(
                    targetColor,
                    new Sort.Color[]{Sort.Color.GREEN, Sort.Color.PURPLE, Sort.Color.PURPLE}
            )){
                new SequentialGroup(
                        Sort.INSTANCE.shootNewGreen(),
                        new WaitUntil(() -> Sort.INSTANCE.shootComplete),
                        Sort.INSTANCE.shootNewPurp(),
                        new WaitUntil(() -> Sort.INSTANCE.shootComplete),
                        Sort.INSTANCE.shootNewPurp()
                ).schedule();

                }
        else if(Arrays.equals(
                targetColor,
                new Sort.Color[]{Sort.Color.PURPLE, Sort.Color.PURPLE, Sort.Color.GREEN}
        )){
                new SequentialGroup(
                        Sort.INSTANCE.shootNewPurp(),
                        new WaitUntil(() -> Sort.INSTANCE.shootComplete),
                        Sort.INSTANCE.shootNewPurp(),
                        new WaitUntil(() -> Sort.INSTANCE.shootComplete),
                        Sort.INSTANCE.shootNewGreen()
                ).schedule();
        }
       else if(Arrays.equals(
                targetColor,
                new Sort.Color[]{Sort.Color.PURPLE, Sort.Color.GREEN, Sort.Color.PURPLE}
        )){
                new SequentialGroup(
                        Sort.INSTANCE.shootNewPurp(),
                        new WaitUntil(() -> Sort.INSTANCE.shootComplete),
                        Sort.INSTANCE.shootNewGreen(),
                        new WaitUntil(() -> Sort.INSTANCE.shootComplete),
                        Sort.INSTANCE.shootNewPurp()
                ).schedule();
        }


            })
            .named("shootInPattern");

    public Command detectTargetColorArray = new LambdaCommand()
            .setStart(()->{
            timer.reset();
            })
            .setUpdate(()->{
                targetColor = Turret.INSTANCE.getColorArray();
            })
            .setIsDone(() -> (timer.seconds()>2)||targetColor!=null)
            .setStop((interrupted)->{
                Turret.INSTANCE.initLimelightSystem();
            });

    public Sort.Color[] getTargetColor(){
        return targetColor;
    }

    @Override
    public void initialize() {
        // initialization logic (runs on init)
    }

    @Override
    public void periodic() {
        colorWeHave = Sort.INSTANCE.getColorArray();
    }


}

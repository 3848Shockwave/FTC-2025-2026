package org.firstinspires.ftc.teamcode.Subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.SubsystemGroup;

public class MySubsystemGroup extends SubsystemGroup {
    public static final MySubsystemGroup INSTANCE = new MySubsystemGroup();

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

    public Command shootInPattern = new LambdaCommand()
            .setStart(() -> {
                for (int i = 0; i < targetColor.length; i++){
                    if (containsColor(targetColor[i])){
                        if (targetColor[i] == Sort.Color.PURPLE){
                            Sort.INSTANCE.shootPurp.schedule();
                        }else{
                            Sort.INSTANCE.shootGreen.schedule();
                        }
                    }
                }
            })
            .setUpdate(()->{
                colorWeHave = Sort.INSTANCE.getColorArray();
            })
            .requires(this)
            .setInterruptible(false)
            .named("shootInPattern");



    @Override
    public void initialize() {
        // initialization logic (runs on init)
    }

    @Override
    public void periodic() {
        colorWeHave = Sort.INSTANCE.getColorArray();
        targetColor = Turret.INSTANCE.getColorArray();
    }


}

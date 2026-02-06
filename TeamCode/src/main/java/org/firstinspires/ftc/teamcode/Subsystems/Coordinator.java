package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

import dev.nextftc.core.commands.Command;

import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.SubsystemGroup;

public class Coordinator extends SubsystemGroup {
    public static final Coordinator INSTANCE = new Coordinator();
    private boolean checkIsDone =false;
    private int counter =0;
    private static ElapsedTime timer = new ElapsedTime();
    private Sort.Color[] colorWeHave = new Sort.Color[3];
    private Sort.Color[] targetColor = new Sort.Color[3];



    private Coordinator() {
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
                        Sort.INSTANCE.shootNewPurp(),
                        new WaitUntil(() -> Sort.INSTANCE.shootComplete)
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
                        Sort.INSTANCE.shootNewGreen(),
                         new WaitUntil(() -> Sort.INSTANCE.shootComplete)
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
                        Sort.INSTANCE.shootNewPurp(),
                        new WaitUntil(() -> Sort.INSTANCE.shootComplete)
                ).schedule();
        }


            }).setInterruptible(true)
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
   public Command tripleLaunch
          = new InstantCommand(() -> {

                  Command[] commands = new Command[6];
                  int counters = 0;
                  for (Sort.Color color : colorWeHave) {
                      if (color != Sort.Color.EMPTY) {
                          if (color == Sort.Color.GREEN) {
                              commands[counters] = Sort.INSTANCE.shootNewGreen();
                              counters++;
                              commands[counters] = new WaitUntil(() -> Sort.INSTANCE.shootComplete);
                              counters++;
                          } else if (color == Sort.Color.PURPLE) {
                              commands[counters] = Sort.INSTANCE.shootNewPurp();
                              counters++;
                              commands[counters] = new WaitUntil(() -> Sort.INSTANCE.shootComplete);
                              counters++;

                          }
                      }
                  }
                  Command[] commandsFinal = Arrays.copyOf(commands, counters);
                if (commandsFinal.length !=0) {
                    new SequentialGroup(commandsFinal
                    ).schedule();
                }

    }).setInterruptible(true)
            .named("tripleLaunch");
//    public Command tripleLaunch = new InstantCommand(() -> {
//        ArrayList<Command> commands = new ArrayList<>();
//        if (colorWeHave == null) return;
//        for (Sort.Color color : colorWeHave) {
//            if (color == null) continue;
//            if (color == Sort.Color.GREEN) {
//                commands.add(Sort.INSTANCE.shootNewGreen());
//                commands.add(new WaitUntil(() -> Sort.INSTANCE.shootComplete));
//            } else if (color == Sort.Color.PURPLE) {
//                commands.add(Sort.INSTANCE.shootNewPurp());
//                commands.add(new WaitUntil(() -> Sort.INSTANCE.shootComplete));
//            }
//        }
//        if (!commands.isEmpty()) {
//            new SequentialGroup(commands.toArray(new Command[0])).schedule();
//        }
//    }).named("tripleLaunch");
   //write a method which takes in an array of commands and iterates through them recusively scheduling them one after another using the .then feature


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
    public void setTargetColor(Sort.Color[] targetColor){
        this.targetColor = targetColor;
    }


}

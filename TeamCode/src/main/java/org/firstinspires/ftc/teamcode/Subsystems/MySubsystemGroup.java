package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.jetbrains.annotations.NotNull;

import java.lang.reflect.Array;
import java.util.ArrayList;
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

    // --- Internal pattern state machine ---
    private enum Action { SHOOT_GREEN, SHOOT_PURP }
    private ArrayList<Action> patternActions = new ArrayList<>();
    private int patternIndex = 0;
    private boolean patternWaitingForComplete = false;
    private boolean patternActive = false;


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
            // Instead of building and scheduling a SequentialGroup here (which would schedule commands from
            // inside a command), we prepare an internal patternActions list and mark the pattern active.
            if(Arrays.equals(
                    targetColor,
                    new Sort.Color[]{Sort.Color.GREEN, Sort.Color.PURPLE, Sort.Color.PURPLE}
            )){
                patternActions.clear();
                patternActions.add(Action.SHOOT_GREEN);
                patternActions.add(Action.SHOOT_PURP);
                patternActions.add(Action.SHOOT_PURP);
                patternIndex = 0;
                patternWaitingForComplete = false;
                patternActive = true;
                return;
                }
        else if(Arrays.equals(
                targetColor,
                new Sort.Color[]{Sort.Color.PURPLE, Sort.Color.PURPLE, Sort.Color.GREEN}
        )){
                patternActions.clear();
                patternActions.add(Action.SHOOT_PURP);
                patternActions.add(Action.SHOOT_PURP);
                patternActions.add(Action.SHOOT_GREEN);
                patternIndex = 0;
                patternWaitingForComplete = false;
                patternActive = true;
                return;
        }
       else if(Arrays.equals(
                targetColor,
                new Sort.Color[]{Sort.Color.PURPLE, Sort.Color.GREEN, Sort.Color.PURPLE}
        )){
                patternActions.clear();
                patternActions.add(Action.SHOOT_PURP);
                patternActions.add(Action.SHOOT_GREEN);
                patternActions.add(Action.SHOOT_PURP);
                patternIndex = 0;
                patternWaitingForComplete = false;
                patternActive = true;
                return;
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
   public Command tripleLaunch
          = new InstantCommand(() -> {
              // Build a list of actions from colorWeHave; do not schedule commands here.
              patternActions.clear();
              if (colorWeHave != null) {
                  for (Sort.Color color: colorWeHave) {
                      if (color == null || color == Sort.Color.EMPTY) continue;
                      if (color == Sort.Color.GREEN) patternActions.add(Action.SHOOT_GREEN);
                      else if (color == Sort.Color.PURPLE) patternActions.add(Action.SHOOT_PURP);
                  }
              }
              if (!patternActions.isEmpty()) {
                  patternIndex = 0;
                  patternWaitingForComplete = false;
                  patternActive = true;
              }


    })
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
        // drive the internal pattern processor which schedules Sort commands as needed (outside of other commands)
        processPattern();
    }

    // Processes the currently-active patternActions array; this runs from periodic() so we're not scheduling
    // commands from within another command. We schedule Sort's InstantCommands from here and wait for
    // Sort.INSTANCE.shootComplete to advance between steps.
    private void processPattern() {
        if (!patternActive) return;
        // if we're in the middle of a step and waiting for completion, check the Sort flag
        if (patternWaitingForComplete) {
            if (Sort.INSTANCE.shootComplete) {
                // consume the completion and advance
                Sort.INSTANCE.shootComplete = false;
                patternWaitingForComplete = false;
                patternIndex++;
                if (patternIndex >= patternActions.size()) {
                    // done
                    patternActive = false;
                    patternActions.clear();
                    patternIndex = 0;
                }
            }
            return;
        }

        // Not currently waiting - start the next action if available
        if (patternIndex < patternActions.size()) {
            Action a = patternActions.get(patternIndex);
            if (a == Action.SHOOT_GREEN) {
                Sort.INSTANCE.shootNewGreen().schedule();
            } else if (a == Action.SHOOT_PURP) {
                Sort.INSTANCE.shootNewPurp().schedule();
            }
            // now wait for Sort to set shootComplete
            patternWaitingForComplete = true;
        } else {
            // nothing to do
            patternActive = false;
            patternActions.clear();
            patternIndex = 0;
        }
    }


}

package org.firstinspires.ftc.teamcode.Subsystems.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.Sort;

import dev.nextftc.core.commands.Command;
import dev.nextftc.ftc.ActiveOpMode;

public class TripleLaunch extends Command {
    private final Command pushBallAndBack;
    private final Command cycleLeft;
    private int step = 0;
    private Command currentCommand = null;

    public TripleLaunch() {
        this.pushBallAndBack = Sort.INSTANCE.pushBallAndBack;
        this.cycleLeft = Sort.INSTANCE.cycleLeft;
        requires(Sort.INSTANCE).setInterruptible(false);
    }

    @Override
    public boolean isDone() {
        return step >= 5 && (currentCommand == null || currentCommand.isDone());
    }

    @Override
    public void start() {
        step = 0;
        currentCommand = null;
        scheduleNext();
    }

    @Override
    public void update() {
        if (currentCommand != null && currentCommand.isDone()) {
            scheduleNext();
        }
    }

    @Override
    public void stop(boolean interrupted) {
        if (currentCommand != null && !currentCommand.isDone()) {
            currentCommand.stop(true);
        }
        currentCommand = null;
    }

    private void scheduleNext() {
        if (step == 0) {
            currentCommand = pushBallAndBack;
           pushBallAndBack.schedule();
            step++;
        } else if (step == 1) {
            currentCommand = cycleLeft;
            cycleLeft.schedule();
            step++;
        } else if (step == 2) {
            currentCommand = pushBallAndBack;
            pushBallAndBack.schedule();
            step++;
        } else if (step == 3) {
            currentCommand = cycleLeft;
            cycleLeft.schedule();
            step++;
        } else if (step == 4) {
            currentCommand = pushBallAndBack;
            pushBallAndBack.schedule();
            step++;
        } else {
            currentCommand = null;
        }
    }
}
package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.SimpleKalmanFilter;

import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class CurrentSensing implements Subsystem {

    public static final CurrentSensing INSTANCE = new CurrentSensing();

    private final MotorEx intake = new MotorEx("intakeMotor").brakeMode();
    private SimpleKalmanFilter currentFilter;

    private static final double JAM_THRESHOLD = 3.8; // Amps
    private static final double REVERSE_POWER = -0.8; // Power to reverse when jam detected
    private static final double NORMAL_POWER = 0.8; // Normal intake power
    private static final long REVERSE_DURATION_MS = 500; // How long to reverse (milliseconds)

    private boolean isReversing = false;
    private long reverseStartTime = 0;
    private double maxCurrentSeen = 0.0;

    // Auto-intake command (schedule to run with jam protection)
    public LambdaCommand autoIntake;

    private CurrentSensing() {
    }

    @Override
    public void initialize() {
        currentFilter = new SimpleKalmanFilter(0.01, 0.2);

        autoIntake = new LambdaCommand()
                .setStart(() -> {
                    isReversing = false;
                    reverseStartTime = 0;
                })
                .setUpdate(() -> {
                    double rawCurrentAmps = intake.getMotor().getCurrent(CurrentUnit.AMPS);
                    double filteredCurrentAmps = currentFilter.filter(rawCurrentAmps);
                    if (rawCurrentAmps > maxCurrentSeen) {
                        maxCurrentSeen = rawCurrentAmps;
                    }

                    if (isReversing) {
                        long elapsedTime = System.currentTimeMillis() - reverseStartTime;
                        if (elapsedTime < REVERSE_DURATION_MS) {
                            intake.setPower(REVERSE_POWER);
                        } else {
                            isReversing = false;
                            intake.setPower(NORMAL_POWER);
                        }
                    } else {
                        if (filteredCurrentAmps > JAM_THRESHOLD) {
                            isReversing = true;
                            reverseStartTime = System.currentTimeMillis();
                            intake.setPower(REVERSE_POWER);
                        } else {
                            intake.setPower(NORMAL_POWER);
                        }
                    }
                })
                .setStop(interrupted -> {
                    isReversing = false;
                })
                .setIsDone(() -> false) // run until explicitly cancelled/overridden
                .setInterruptible(true)
                .named("autoIntake");
    }

    @Override
    public void periodic() {
        // No periodic actions required; autoIntake handles runtime behavior when scheduled.
    }

    // Utility getters
    public boolean isReversing() { return isReversing; }
    public double getMaxCurrentSeen() { return maxCurrentSeen; }
    public double getFilteredCurrent() { return currentFilter.filter(intake.getMotor().getCurrent(CurrentUnit.AMPS)); }
    public double getRawCurrent() { return intake.getMotor().getCurrent(CurrentUnit.AMPS); }
    public double getIntakePower() {
        return intake.getPower();
    }

    /** Returns true when intake motor is driving (forward or reverse) beyond a small deadband. */
    public boolean isIntakeActive() {
        return Math.abs(intake.getPower()) > 0.05;
    }
}
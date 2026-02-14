package org.firstinspires.ftc.teamcode.Tests;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "Current Sensing", group = "Testing")
public class CurrentSensing extends NextFTCOpMode {

    MotorEx intake;
    SimpleKalmanFilter currentFilter;

    private static final double JAM_THRESHOLD = 4.0; // Amps
    private static final double REVERSE_POWER = -0.8; // Power to reverse when jam detected
    private static final double NORMAL_POWER = 1.0; // Normal intake power
    private static final long REVERSE_DURATION_MS = 500; // How long to reverse (milliseconds)

    private boolean isReversing = false;
    private long reverseStartTime = 0;
    private boolean manualForward = false;
    private boolean manualReverse = false;

    double maxCurrentSeen = 0.0;

    public CurrentSensing() {
        addComponents(BindingsComponent.INSTANCE);
    }

    @Override
    public void onInit() {
        intake = new MotorEx("intakeMotor");
        intake.brakeMode();
        currentFilter = new SimpleKalmanFilter(0.01, 0.1);
    }

    @Override
    public void onStartButtonPressed() {
        Button x_button = button(() -> gamepad1.x)
                .whenBecomesTrue(() -> maxCurrentSeen = 0.0);
    }

    @Override
    public void onUpdate() {

        double rawCurrentAmps = intake.getMotor().getCurrent(CurrentUnit.AMPS);

        double filteredCurrentAmps = currentFilter.filter(rawCurrentAmps);

        if (rawCurrentAmps > maxCurrentSeen) {
            maxCurrentSeen = rawCurrentAmps;
        }


        manualForward = gamepad1.dpad_up;
        manualReverse = gamepad1.dpad_down;


        if (manualForward) {
            intake.setPower(NORMAL_POWER);
            telemetry.addData("Status", "MANUAL FORWARD");
        }
        else if (manualReverse) {
            intake.setPower(REVERSE_POWER);
            telemetry.addData("Status", "MANUAL REVERSE");
        }

        else {
            if (isReversing) {
                long elapsedTime = System.currentTimeMillis() - reverseStartTime;
                if (elapsedTime < REVERSE_DURATION_MS) {
                    intake.setPower(REVERSE_POWER);
                    telemetry.addData("Status", "AUTO REVERSING (%.1f s remaining)",
                        (REVERSE_DURATION_MS - elapsedTime) / 1000.0);
                } else {
                    isReversing = false;
                    intake.setPower(NORMAL_POWER);
                    telemetry.addData("Status", "AUTO NORMAL");
                }
            } else {
                if (filteredCurrentAmps > JAM_THRESHOLD) {
                    isReversing = true;
                    reverseStartTime = System.currentTimeMillis();
                    intake.setPower(REVERSE_POWER);
                    telemetry.addData("Status", "JAM DETECTED - AUTO REVERSING");
                } else {
                    intake.setPower(NORMAL_POWER);
                    telemetry.addData("Status", "AUTO NORMAL");
                }
            }
        }


        telemetry.addData("1. Real-time Current", "%.2f A", rawCurrentAmps);
        telemetry.addData("2. MAX Current Seen", "%.2f A", maxCurrentSeen);
        telemetry.addData("3. Filtered Current", "%.2f A", filteredCurrentAmps);
        telemetry.addData("4. Threshold", "%.2f A", JAM_THRESHOLD);
        telemetry.addLine("");
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("D-Pad Up: Manual Forward");
        telemetry.addLine("D-Pad Down: Manual Reverse");
        telemetry.addLine("X Button: Reset Max Current");
        telemetry.addLine("(Auto jam detection when no manual input)");


        telemetry.update();
    }
}
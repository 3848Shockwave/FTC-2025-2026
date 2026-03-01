package org.firstinspires.ftc.teamcode.Tests;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

@TeleOp(name = "Scissor Lift Center", group = "Test")
public class ScissorLiftCenter extends NextFTCOpMode {

    public static final ScissorLiftSubsystem scissorLift = new ScissorLiftSubsystem();

    private boolean isUp = false;

    public ScissorLiftCenter() {
        addComponents(
                new SubsystemComponent(scissorLift),
                BindingsComponent.INSTANCE,
                BulkReadComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        telemetry.addData("Status", "Initialized - Scissor Lift Test");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // DPAD UP -> Move scissor lift up
        Button upButton = button(() -> gamepad1.dpad_up)
                .whenBecomesTrue(() -> {
                    scissorLift.moveUp().schedule();
                    isUp = true;
                });

        // DPAD DOWN -> Move scissor lift down
        Button downButton = button(() -> gamepad1.dpad_down)
                .whenBecomesTrue(() -> {
                    scissorLift.moveDown().schedule();
                    isUp = false;
                });

        // A button -> Toggle up/down
        Button toggleButton = button(() -> gamepad1.a)
                .whenBecomesTrue(() -> {
                    if (isUp) {
                        scissorLift.moveDown().schedule();
                    } else {
                        scissorLift.moveUp().schedule();
                    }
                    isUp = !isUp;
                });
    }

    @Override
    public void onUpdate() {
        telemetry.addData("Scissor Lift Position", isUp ? "UP" : "DOWN");
        telemetry.addData("Left Servo Pos", scissorLift.getLeftPosition());
        telemetry.addData("Right Servo Pos", scissorLift.getRightPosition());
        telemetry.update();
    }

    public static class ScissorLiftSubsystem implements Subsystem {

        // Adjust these servo names to match your hardware configuration
        private final ServoEx scissorServoLeft = new ServoEx("scissorLeft");
        private final ServoEx scissorServoRight = new ServoEx("scissorRight");

        // Adjust these positions to match your scissor lift's up/down positions
        private static final double UP_POSITION_LEFT = 1;
        private static final double DOWN_POSITION_LEFT = 0.6

                ;
        private static final double UP_POSITION_RIGHT = 0.34;
        private static final double DOWN_POSITION_RIGHT =  .8;

        public Command moveUp() {
            return new ParallelGroup(
                    new SetPosition(scissorServoLeft, UP_POSITION_LEFT),
                    new SetPosition(scissorServoRight, UP_POSITION_RIGHT)
            );
        }

        public Command moveDown() {
            return new ParallelGroup(
                    new SetPosition(scissorServoLeft, DOWN_POSITION_LEFT),
                    new SetPosition(scissorServoRight, DOWN_POSITION_RIGHT)
            );
        }

        public double getLeftPosition() {
            return scissorServoLeft.getPosition();
        }

        public double getRightPosition() {
            return scissorServoRight.getPosition();
        }

        @Override
        public void initialize() {
            // Start in the down position
            scissorServoLeft.setPosition(DOWN_POSITION_LEFT);
            scissorServoRight.setPosition(DOWN_POSITION_RIGHT);
        }

        @Override
        public void periodic() {
            // No periodic logic needed
        }
    }
}
package org.firstinspires.ftc.teamcode.Tests;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Sort;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.components.BindingsComponent;

import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.bindings.Button;
import dev.nextftc.hardware.impl.ServoEx;
@TeleOp(name = "PTOTest", group = "Testing")
public class PTOLiftTest extends NextFTCOpMode {
    MotorEx leftMotor = new MotorEx("leftMotor").brakeMode();
    MotorEx rightMotor = new MotorEx("rightMotor").brakeMode();
    ServoEx leftServo = new ServoEx("leftServo");
    ServoEx rightServo = new ServoEx("rightServo");

    Button x_button,y_button;

    public PTOLiftTest() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    @Override
    public void onInit() {
       x_button = button(() -> gamepad1.x);
        y_button = button(() -> gamepad1.y);
       y_button.whenBecomesTrue(()->{
           leftServo.setPosition(.5);
           rightServo.setPosition(.5);
       });
       x_button.whenTrue(()->{
           leftMotor.setPower(1);
           rightMotor.setPower(1);
           leftServo.setPosition(.5);
           rightServo.setPosition(.5);
       });
        x_button.whenFalse(()->{
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        });
    }



    @Override
    public void onUpdate() {

    }
}

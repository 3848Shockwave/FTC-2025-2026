package org.firstinspires.ftc.teamcode.Tests;
import static dev.nextftc.bindings.Bindings.button;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import dev.nextftc.control.ControlSystem;

import org.firstinspires.ftc.teamcode.Subsystems.Sort;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.bindings.Button;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.builder.FilterBuilder;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;

import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
@Configurable
@TeleOp(name="ELC Encoder V2 Test", group="Sensor")
public class ELCEncoderTeleOp extends NextFTCOpMode {
    private ELCEncoderV2 encoder;
    private CRServo servoRight;
    private CRServo servoLeft;
    KineticState newGoal ;
    double goal = 0.0;
    Button aButton;
    public static double kp = 0.00;
    public static double ki = 0.00;
    public static double kd = 0.00;
    public static double kv = 0.00;
    public static double ka = 0.00;
    public static double ks = 0.00;

    public ELCEncoderTeleOp() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    TelemetryManager telemetryManager ;
    ControlSystem servoControl = null;
    @Override
    public void onInit() {
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        aButton = button(() -> gamepad1.a);
        encoder = new ELCEncoderV2(hardwareMap, "encoder");
        servoRight = hardwareMap.get(CRServo.class, "servo_right");
        servoLeft = hardwareMap.get(CRServo.class, "servo_left");
        servoControl   = dev.nextftc.control.ControlSystem.builder()
                .posPid(kp,ki,kd)
                .posFilter(filter->filter.lowPass(.3))
                .basicFF(kv,ka,ks)
                .build();
        aButton.whenBecomesTrue(()->{
            servoControl   = dev.nextftc.control.ControlSystem.builder()
                    .posPid(kp,ki,kd)
                    .posFilter(filter->filter.lowPass(.3))
                    .basicFF(kv,ka,ks)
                    .build();
            goal+=120;
            servoControl.setGoal( new KineticState(goal));
        });
    }

    @Override
    public void onUpdate() {

        encoder.updateRotations();
        telemetryManager.addData("Raw Voltage", String.format("%.3f V", encoder.getVoltage()));
        telemetryManager.addData("Max Voltage", String.format("%.3f V", encoder.getMaxVoltage()));
        telemetryManager.addData("", "");
        telemetryManager.addData("Absolute Position", String.format("%.2f°", encoder.getDegrees()));
        telemetryManager.addData("Position (Radians)", String.format("%.3f rad", encoder.getRadians()));
        telemetryManager.addData("Percentage", String.format("%.1f%%", encoder.getPercentage()));
        telemetryManager.addData("", "");
        telemetryManager.addData("=== ROTATION TRACKING ===", "");
        telemetryManager.addData("Rotations", encoder.getRotations());
        telemetryManager.addData("Total Degrees", String.format("%.2f°", encoder.getTotalDegrees()));
        telemetryManager.addData("Total Radians", String.format("%.3f rad", encoder.getTotalRadians()));
        telemetryManager.addData("Goal", goal);
        if(newGoal!=null) {
            telemetryManager.addData("newGoal", newGoal);
        }
        telemetryManager.addData("controlsystemgoal" ,servoControl.getGoal());
       // telemetryManager.addData("calcpwer",servoControl.calculate(encoder.getState()));
        telemetryManager.addData("error",Math.abs(goal-encoder.getTotalDegrees()));
        telemetryManager.addData("encstate",encoder.getState());



        double servoPower =servoControl.calculate(encoder.getState());
        if(servoPower>1){
            servoPower=1;
        }
        if(servoPower<-1){
            servoPower=-1;
        }
        if(!servoControl.isWithinTolerance(new KineticState(4))) {
            servoRight.setPower(-servoPower);
            servoLeft.setPower(-servoPower);
            telemetryManager.update(telemetry);
        }


    }
}


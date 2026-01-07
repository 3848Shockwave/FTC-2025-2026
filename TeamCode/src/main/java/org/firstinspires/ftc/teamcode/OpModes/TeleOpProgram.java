package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import dev.nextftc.bindings.Button;
import dev.nextftc.bindings.Range;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.impl.MotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.Sort;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import static dev.nextftc.bindings.Bindings.button;
import static dev.nextftc.bindings.Bindings.range;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;//most important one to import


@Configurable
@TeleOp(name = "TeleOp Program", group = "Production")
public class TeleOpProgram extends NextFTCOpMode {
    private final Pose BstartPose = new Pose(70, 86, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose RstartPose = new Pose(96, 86, Math.toRadians(270)); // Start Pose of our robot.
    MotorEx intake = new MotorEx("intake").brakeMode();







    private boolean sideSelected = false;
    public static double kp =0.004;
    public static double ki = 0.004;
    public static double kd =0.00026;
    public static double kf = 0.0000275;
    public static double power = 1.0;

    public TeleOpProgram(){
        addComponents(
                new SubsystemComponent(Sort.INSTANCE),
                new SubsystemComponent(Turret.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
                );
    }


    private boolean motorToggle = false;
    private boolean launchToggle = false;



    private TelemetryManager telemetryManager;

    @Override
    public void onInit() {
       // follower().setPose(startPose);
        Turret.INSTANCE.resetRotateMotorPosition();
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryManager.addData("Select Alliance","");
        telemetryManager.update(telemetry);
        if(!ActiveOpMode.isStarted()&&!sideSelected) {
            Button x_button = button(() -> gamepad1.x).whenBecomesTrue(() ->{
                Turret.INSTANCE.setSide("blue");
                telemetryManager.addData("Alliance: ","Blue");
                telemetryManager.update(telemetry);
                follower().setPose(BstartPose);
                sideSelected=true;
            });

            Button b_button = button(() -> gamepad1.b).whenBecomesTrue(() ->
            {
                Turret.INSTANCE.setSide("red");
                telemetryManager.addData("Alliance: ","Red");
                telemetryManager.update(telemetry);
                follower().setPose(RstartPose);
                sideSelected=true;
            });
        }
        Sort.INSTANCE.checkColors();
       // Turret.INSTANCE.limelightProcessing.getLimelightStatus();
    }

    @Override
    public void onStartButtonPressed(){
        Button a_button = button(() -> gamepad1.a).whenBecomesTrue(() ->
        {
            motorToggle = !motorToggle;
            if (motorToggle) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }
        });

        Button b_button = button(() -> gamepad1.b).whenBecomesTrue(()->
        {
            motorToggle = !motorToggle;
            if (motorToggle) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }
        });

//        Button x_button = button(() -> gamepad1.x).whenBecomesTrue(()->
//        {
//            launchToggle = !launchToggle;
//            if (launchToggle) {
//                Turret.INSTANCE.setLaunchMotorSpeed(1.0);
//            } else {
//                Turret.INSTANCE.setLaunchMotorSpeed(0.0);
//            }
//        });
        Button y_button = button(() -> gamepad1.y)
                .whenBecomesTrue(Turret.INSTANCE::resetRotateMotorPosition);

        Button dpad_up = button(() -> gamepad1.dpad_up)
                .whenBecomesTrue(Sort.INSTANCE.pushBallAndBack);
        Button dpad_down = button(() -> gamepad1.dpad_down)
                .whenBecomesTrue(Sort.INSTANCE.tripleLaunch);
       // Button dpad_down = button(() -> gamepad1.dpad_down).whenBecomesTrue(Sort.INSTANCE.cycleRight.thenWait(.5).then(Sort.INSTANCE.cycleLeft));

        Button left_bumper = button(() -> gamepad1.left_bumper)
                .whenBecomesTrue(Sort.INSTANCE.cycleLeft);


        Button right_bumper = button(() -> gamepad1.right_bumper)
                .whenBecomesTrue(Sort.INSTANCE.cycleRight);

        Button right_trigger = range(() -> gamepad1.right_trigger)
                .greaterThan(0.2)
                .whenBecomesTrue(Sort.INSTANCE.loadGreen);

        Button left_trigger = range(() -> gamepad1.left_trigger)
                .greaterThan(0.2)
                .whenBecomesTrue(Sort.INSTANCE.loadPurp);




        follower().startTeleopDrive();

        DriverControlledCommand driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
//                range(() -> {
//                    double v = Gamepads.gamepad1().leftStickY().get();
//                    double exp = Math.copySign(Math.pow(Math.abs(v), 3.6), v);
//                    return -exp;
//                }),
//                range(() -> {
//                    double v = Gamepads.gamepad1().leftStickX().get();
//                    double exp = Math.copySign(Math.pow(Math.abs(v), 3.6), v);
//                    return -exp;
//                }),
                Gamepads.gamepad1().rightStickX().negate(),
                false
        );
        driverControlled.schedule();


    }

    @Override
    public void onUpdate() {
        double turretPos = Turret.INSTANCE.getRotateMotorPosition();
        double turretWant = Turret.INSTANCE.getRotateMotorPosition()+Turret.INSTANCE.calculatePosition();
        // Turret.INSTANCE.setSetVelocity(newVelocity);
        Turret.INSTANCE.rebuildControlSystem(kp, ki, kd, kf,power);
       // Sort.INSTANCE.rebuildControlSystem(kp,ki,kd,kf,power);
        telemetryManager.addData("SpindexMotorPosition", Sort.INSTANCE.getCurrentPosition());
       if(Sort.INSTANCE.getSpinLimitSwitchStatus()){
           telemetryManager.addData("FIRE READY","");
       }
       else{
              telemetryManager.addData("FIRE NOT READY","");
       }
        telemetryManager.addData("turretMotorPosition", turretPos);
        telemetryManager.addData("SpinNextPosition", Sort.INSTANCE.getTargetPosition() );
        telemetryManager.addData("TurretNextPosition", turretWant );
        telemetryManager.addData("error", Math.abs(Sort.INSTANCE.getCurrentPosition()-Sort.INSTANCE.getTargetPosition()) );
        telemetryManager.addData("Colors",Sort.INSTANCE.getColorArray());
        telemetryManager.addData("Pipeline", Turret.INSTANCE.limelightProcessing.getCurrentPipeline());
        telemetryManager.addData("Limelight Status", Turret.INSTANCE.limelightProcessing.limelightTelemetry());
        telemetryManager.addData("Alliance",Turret.INSTANCE.getSide());
        telemetryManager.addData("turretVelocity",Turret.INSTANCE.getTurretVelocity());
       // telemetryManager.addData("desiredVelocity",newVelocity);
        telemetryManager.update(telemetry);

    }


}
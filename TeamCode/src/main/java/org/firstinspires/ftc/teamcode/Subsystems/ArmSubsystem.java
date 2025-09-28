package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmSubsystem extends SubsystemBase {
    private DcMotor Lift;
    private DcMotor Extend;

    public ArmSubsystem(HardwareMap hardwareMap) {
        Lift = hardwareMap.get(DcMotor.class, "Lift");
        Extend = hardwareMap.get(DcMotor.class, "Extend");

        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Lift.setDirection(DcMotor.Direction.FORWARD);
        Extend.setDirection(DcMotor.Direction.FORWARD);

        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Control the arm lift movement
     * @param power motor power (-1.0 to 1.0)
     */
    public void liftArm(double power) {
        Lift.setPower(power);
    }

    public void extendArm(double power) {
        Extend.setPower(power);
    }

    /**
     * Get the current position of the lift motor
     * @return lift motor encoder count
     */
    public int getLiftPosition() {
        return Lift.getCurrentPosition();
    }

    public int getExtendPosition() {
        return Extend.getCurrentPosition();
    }

    public void stop() {
        Lift.setPower(0);
        Extend.setPower(0);
    }
}
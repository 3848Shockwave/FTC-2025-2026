package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;


import com.qualcomm.robotcore.hardware.DcMotor;      // Motor control
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmSubsystem extends SubsystemBase {
    private DcMotor Lift;
    private DcMotor Extend;

    //Object for this class
    public ArmSubsystem(HardwareMap hardwareMap) {
        Lift = hardwareMap.get(DcMotor.class, "Lift");       //start bot last year has Gobilda motor: 5203-2402-0051, ppr 1425.1
        Extend = hardwareMap.get(DcMotor.class, "Extend");   //start bot las year has Gobilda motor: 5203-2402-0019, ppr 537.7

        //enable encoders
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set direction for motors if needed
        Lift.setDirection(DcMotor.Direction.FORWARD);
        Extend.setDirection(DcMotor.Direction.FORWARD);

        // Set zero power behavior to brake
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    /**
     * 控制机械臂升降
     * power 电机功率 (-1.0 到 1.0)
     */
    public void liftArm(double power) {
        Lift.setPower(power);
    }


    public void extendArm(double power) {
        Extend.setPower(power);
    }

    /**
     * 获取升降电机当前位置
     * @return 升降电机编码器计数
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

    /*
    *  cpr = ppr mutiple by 4
        for methods setTargetPosition(int Position) the position refers to the number of encoder counts
     */
}
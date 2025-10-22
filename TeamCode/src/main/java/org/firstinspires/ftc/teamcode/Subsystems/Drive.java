package org.firstinspires.ftc.teamcode.Subsystems;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
public class Drive implements Subsystem {

    public static final Drive INSTANCE = new Drive();
    public Drive(){}

    private final MotorEx frontLeft = new MotorEx("front_left").reversed().brakeMode();
    private final MotorEx frontRight = new MotorEx("front_right").brakeMode();
    private final MotorEx backLeft = new MotorEx("back_left").reversed().brakeMode();
    private final MotorEx backRight = new MotorEx("back_right").brakeMode();


    public void setMotorPowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

}

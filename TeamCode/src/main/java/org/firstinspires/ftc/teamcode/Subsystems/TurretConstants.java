package org.firstinspires.ftc.teamcode.Subsystems;

public class TurretConstants {
    /*
    13.7 : 1 Ratio, 435 RPM
    Encoder Resolution	384.5 PPR at the Output Shaft
    Encoder Resolution Formula	((((1+(46/17))) * (1+(46/17))) * 28)

    sku:5203-2402-0014 https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-6mm-d-shaft-435-rpm-36mm-gearbox-3-3-5v-encoder/

    24:135 (gear ratio for the turning plate)
    217.572 mm --> diameter for the plate --> plate Circumference:68.3523 cm

    Total Gear Ratio:
    13.7 * 135/24 = 77.0625

    total counts per plate revolution:
    77.0625 * 384.5 = 29630.53125 counts/plate rev

    __distance moved per encoder count__
    =68.3523/29630.53125 = 0.0023068267
 */
    public static final double distancePerImpulseForRotation = 0.0023068267;//cm
    public static final double rotationDiameter = 217.572;
    public static final double rotateMotorPPR = 384.5;
    public static final double TGearRatioRotation = 77.0625;



    /*
    19.2:1 Ratio, 312 RPM
     */
    public static final double distanceperImulseForLunch = 0.00292132;



    public static final double gravityAccalerationValue = 98.1;//cm/s^2

    public static final double flyWheelDiameter = 96;
    public static final double motorShaftRadiusForLuncher = 4;


}

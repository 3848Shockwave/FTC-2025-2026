package org.firstinspires.ftc.teamcode.Subsystems;

public class TurretConstants {
    /*
    13.7 : 1 Ratio, 435 RPM
    Encoder Resolution	384.5 PPR at the Output Shaft
    Encoder Resolution Formula	((((1+(46/17))) * (1+(46/17))) * 28)

    sku:5203-2402-0014 https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-6mm-d-shaft-435-rpm-36mm-gearbox-3-3-5v-encoder/

    24:135 (gear ratio for the turning plate)
    217.572 mm --> diameter for the plate --> plate Circumference:68.3523 cm

    Total Gear Ratio from motor shaft to plate:
    135/24 = 77.0625 Output shaft encoder, so no need to multiply by 13.7,

    //Encoder ticks per plate rotation:
    135/24* 384.5 = 2162.8125

    __distance moved per encoder count__
    =68.3523/2162.8125 = 0.03160343302
    This represents the degrees that the output 135 tooth pulley moves per encoder tick
 */
    public static final double distancePerImpulseForRotation = 0.03160343302;
    public static final double rotationDiameter = 217.572;
    public static final double rotateMotorPPR = 384.5;
    public static final double TGearRatioRotation = 77.0625;

    public static final double ticksPerDegreeOfRotation = 6.0078125;


    /*
    19.2:1 Ratio, 312 RPM
     */
    public static final double distanceperImulseForLunch = 0.00292132;



    public static final double gravityAccalerationValue = 98.1;//cm/s^2

    public static final double flyWheelDiameter = 96;
    public static final double motorShaftRadiusForLuncher = 4;


}
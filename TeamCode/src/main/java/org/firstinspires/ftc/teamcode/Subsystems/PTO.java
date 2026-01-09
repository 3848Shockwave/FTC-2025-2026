package org.firstinspires.ftc.teamcode.Subsystems;

import java.util.Set;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.positionable.SetPositions;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.hardware.powerable.SetPower;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

public class PTO implements Subsystem {
    public static final PTO INSTANCE = new PTO();

    private final MotorEx leftFront = new MotorEx("front_left");
    private final MotorEx rightFront = new MotorEx("front_right");

    private final ServoEx linearServoL = new ServoEx("linearServoL");
    private final ServoEx linearServoR = new ServoEx("linearServoR");

    public Command engageL = new SetPosition(linearServoL, 0.5).requires(linearServoL);
    public Command engageR = new SetPosition(linearServoR, 0.5).requires(linearServoR);
    public Command engage = new ParallelGroup(engageL, engageR).requires(linearServoL, linearServoR);

    public Command liftR = new SetPower(rightFront, 1);
    public Command liftL = new SetPower(leftFront, 1);

    public Command lift = new ParallelGroup(liftL, liftR).requires(leftFront, rightFront);



}

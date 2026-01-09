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

    private final MotorEx leftBack = new MotorEx("back_left");
    private final MotorEx rightBack = new MotorEx("back_right");

    private final ServoEx linearServoL = new ServoEx("liftServoLeft");
    private final ServoEx linearServoR = new ServoEx("liftServoRight");

    public Command engageL = new SetPosition(linearServoL, 0.5).requires(linearServoL);
    public Command engageR = new SetPosition(linearServoR, 0.5).requires(linearServoR);
    public Command engage = new ParallelGroup(engageL, engageR).requires(linearServoL, linearServoR);

    public Command liftLeft = new SetPower(leftBack, 1.0);
    public Command liftRight = new SetPower(rightBack, -1.0);
    public Command stopLeft = new SetPower(leftBack, 0.0);
    public Command stopRight = new SetPower(rightBack, 0.0);



}

package org.firstinspires.ftc.teamcode.Subsystems;

import dev.nextftc.core.subsystems.SubsystemGroup;

public class MySubsystemGroup extends SubsystemGroup {
    public static final MySubsystemGroup INSTANCE = new MySubsystemGroup();

    private MySubsystemGroup() {
        super(
                Turret.INSTANCE,
                Sort.INSTANCE
        );

    }
}

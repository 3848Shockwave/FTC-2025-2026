package org.firstinspires.ftc.teamcode.Subsystems.Helpers;

import com.pedropathing.geometry.Pose;

public final class RobotConfig {
    public static Alliance alliance = null;
    public static Pose finalMeasuredPose = null;
    public static AutonomousStartEndPoses autonomousStartEndPoses = null;
    private RobotConfig() {
    }
    public enum Alliance {
        RED,
        BLUE
    }

    public enum AutonomousStartEndPoses {
        GOALSIDERED(new Pose(124, 128, Math.toRadians(225)), new Pose(78, 110, Math.toRadians(92))),
        GOALSIDEBLUE(new Pose(20, 128, Math.toRadians(315)), new Pose(44, 110, Math.toRadians(92))),
        FARSIDERED(new Pose(84, 10, Math.toRadians(90)), new Pose(96, 86, Math.toRadians(270))),
        FARSIDEBLUE(new Pose(60, 10, Math.toRadians(90)), new Pose(70, 86, Math.toRadians(90)));
        private final Pose startPose;
        private final Pose endPose;

        AutonomousStartEndPoses(Pose startPose, Pose endPose) {
            this.startPose = startPose;
            this.endPose = endPose;
        }

        public Pose getStartPose() {
            return startPose;
        }

        public Pose getEndPose() {
            return endPose;
        }

    }


}

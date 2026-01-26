package org.firstinspires.ftc.teamcode.Subsystems.Helpers;

import com.pedropathing.geometry.Pose;

public final class RobotConfig {
    public static Alliance alliance = null;
    public static Pose finalMeasuredPose = null;
    public static double finalMeasuredSpindexPosition = 0;
    public static AutonomousStartEndPoses autonomousStartEndPoses = null;
    public static RobotStateTracker robotStateTracker = null;
    private RobotConfig() {
    }
    public enum Alliance {
        RED,
        BLUE
    }

    public enum AutonomousStartEndPoses {
        GOALSIDERED(new Pose(124, 122, Math.toRadians(217)), new Pose(78, 110, Math.toRadians(80))),
        GOALSIDEBLUE(new Pose(20, 128, Math.toRadians(315)), new Pose(44, 110, Math.toRadians(92))),
        FARSIDEREDMOVE(new Pose(84, 8.5, Math.toRadians(270)), new Pose(109.387, 19.634, Math.toRadians(180))),
        FARSIDEREDSCORE(new Pose(84, 8.5, Math.toRadians(270)), new Pose(84.435, 83.670, Math.toRadians(225))),
        FARSIDEBLUEMOVE(new Pose(59.2, 8.5, Math.toRadians(270)), new Pose(34.749, 20.953, Math.toRadians(180))),
        FARSIDEBLUESCORE(new Pose(59.2, 8.5, Math.toRadians(270)), new Pose(47.670, 95.450,Math.toRadians(315)));

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

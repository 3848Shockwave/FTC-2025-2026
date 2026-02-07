package org.firstinspires.ftc.teamcode.Subsystems.Helpers;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Subsystems.Sort;

public final class RobotConfig {
    public static Alliance alliance = null;
    public static Pose finalMeasuredPose = null;
    public static double finalMeasuredSpindexPosition = 0;
    public static Sort.Color[] finalMeasuredColors = null;
    public static AutonomousStartEndPoses autonomousStartEndPoses = null;
    public static RobotStateTracker robotStateTracker = null;
    private RobotConfig() {
    }
    public enum Alliance {
        RED,
        BLUE
    }

    public enum AutonomousStartEndPoses {
        GOALSIDEREDMOVE(new Pose(123, 123, Math.toRadians(37)), new Pose(86.316, 111.789, Math.toRadians(205))),
        GOALSIDEREDSCORE(new Pose(123, 123, Math.toRadians(37)), new Pose(96.310, 96.825, Math.toRadians(230))),
        GOALSIDEBLUEMOVE(new Pose(20.75, 123.5, Math.toRadians(143)), new Pose(59.391, 111.989, Math.toRadians(270))),
        GOALSIDEBLUESCORE(new Pose(20.75, 123.5, Math.toRadians(143)), new Pose(46.5, 96.5, Math.toRadians(315))),
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

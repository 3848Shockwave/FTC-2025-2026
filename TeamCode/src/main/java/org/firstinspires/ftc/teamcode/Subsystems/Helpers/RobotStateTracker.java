package org.firstinspires.ftc.teamcode.Subsystems.Helpers;

import com.pedropathing.geometry.Pose;

public class RobotStateTracker {
    Pose lastMeasuredPose = null;
    boolean liftActive = false;
    //if we are lifting the robot, NOTHING will be able to move. this will be set ONCE, and can't be reset during the match.
    boolean cantSee = false;

    // this is for jerry if you want we can use this to track if we
    // actually can see the aprilTag. if we can't we can use the other logic for just aiming at apriltag
    public RobotStateTracker() {

    }

    public Pose getLastMeasuredPose() {
        return lastMeasuredPose;
    }

    public void updateLastPose(Pose currentPose) {
        lastMeasuredPose = currentPose;
    }

    public void setLiftActive() {
        liftActive = true;
    }

    public boolean getLiftActive() {
        return liftActive;
    }

    public boolean getBlind() {
        return cantSee;
    }

    public void setBlind(boolean ahhimblind) {
        cantSee = ahhimblind;
    }

}

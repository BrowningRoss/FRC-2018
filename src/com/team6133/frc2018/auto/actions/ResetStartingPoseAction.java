package com.team6133.frc2018.auto.actions;

import com.team6133.frc2018.subsystems.Drive;
import com.team6133.lib.util.math.Rotation2d;

public class ResetStartingPoseAction extends RunOnceAction {
    private Rotation2d mStartingPose;

    public ResetStartingPoseAction(Rotation2d pose) {
        mStartingPose = pose;
    }

    @Override
    public synchronized void runOnce() {
        Drive.getInstance().setGyroAngle(mStartingPose);
    }
}

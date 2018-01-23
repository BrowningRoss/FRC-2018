package com.team6133.frc2018.auto.actions;

import com.team6133.frc2018.paths.PathContainer;
import com.team6133.frc2018.subsystems.Drive;
import com.team6133.lib.util.math.RigidTransform2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * Resets the robot's current pose based on the starting pose stored in the
 * pathContainer object.
 *
 * @see PathContainer
 * @see Action
 * @see RunOnceAction
 */
public class ResetPoseFromPathAction extends RunOnceAction {

    protected PathContainer mPathContainer;

    public ResetPoseFromPathAction(PathContainer pathContainer) {
        mPathContainer = pathContainer;
    }

    @Override
    public synchronized void runOnce() {
        RigidTransform2d startPose = mPathContainer.getStartPose();
        Drive.getInstance().setGyroAngle(startPose.getRotation());
    }
}
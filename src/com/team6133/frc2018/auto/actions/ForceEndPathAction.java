package com.team6133.frc2018.auto.actions;

import com.team6133.frc2018.subsystems.Drive;
import com.team6133.lib.util.DriveSignal;

/**
 * Forces the current path the robot is driving on to end early
 *
 * @see DrivePathAction
 * @see Action
 * @see RunOnceAction
 */
public class ForceEndPathAction extends RunOnceAction {

    @Override
    public synchronized void runOnce() {
        Drive.getInstance().setOpenLoop(DriveSignal.NEUTRAL);
    }
}
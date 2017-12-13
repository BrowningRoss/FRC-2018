package com.team6133.frc2018.auto.actions;

import com.team6133.frc2018.paths.PathContainer;
import com.team6133.frc2018.subsystems.Drive;
import com.team6133.lib.util.control.Path;

/**
 * Drives the robot along the Path defined in the PathContainer object. The
 * action finishes once the robot reaches the end of the path.
 * 
 * @see PathContainer
 * @see Path
 * @see Action
 */
public class DrivePathAction implements Action {

	private PathContainer mPathContainer;
	private Path mPath;
	private Drive mDrive = Drive.getInstance();

	public DrivePathAction(PathContainer p) {
		mPathContainer = p;
		mPath = mPathContainer.buildPath();
	}

	@Override
	public boolean isFinished() {
		return mDrive.isDoneWithPath();
	}

	@Override
	public void update() {
		// Nothing done here, controller updates in mEnabedLooper in robot
	}

	@Override
	public void done() {
		// TODO: Perhaps set wheel velocity to 0?
	}

	@Override
	public void start() {
		mDrive.setWantDrivePath(mPath, mPathContainer.isReversed());
	}
}
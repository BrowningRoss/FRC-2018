package com.team6133.frc2018.auto.actions;

import com.team6133.frc2018.auto.AutonPathSettings;
import com.team6133.frc2018.subsystems.Drive;
import com.team6133.lib.util.SensorTarget;
import edu.wpi.first.wpilibj.Timer;

public class DrivePathAction implements Action {
    private AutonPathSettings mPath;
    private Drive mDrive = Drive.getInstance();

    private boolean mIsFinished = false;

    public DrivePathAction(AutonPathSettings path) {
        mPath = path;
        if (mPath.getSensorTarget().sensor != SensorTarget.Sensor.Ultra) {
            mDrive.setVoltageBothIRPD(mPath.getSensorTarget().target);
        }
    }

    @Override
    public boolean isFinished() {
        return mIsFinished;
    }

    @Override
    public void update() {
        mIsFinished = mDrive.updateAutonPath();
    }

    @Override
    public void done() {

    }

    @Override
    public void start() {
        mDrive.setAutonPath(mPath, Timer.getFPGATimestamp());
    }
}

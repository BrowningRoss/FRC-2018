package com.team6133.frc2018.auto.actions;

import com.team6133.frc2018.Constants;
import com.team6133.frc2018.auto.AutonPathSettings;
import com.team6133.frc2018.subsystems.Drive;
import com.team6133.lib.util.SensorTarget;
import edu.wpi.first.wpilibj.Timer;

public class DrivePathAction implements Action {
    private AutonPathSettings mPath;
    private Drive mDrive = Drive.getInstance();
    private double mStartTime;

    private boolean mIsFinished = false;

    public DrivePathAction(AutonPathSettings path) {
        mPath = path;
        double min, max;
        if (path.getInvertIRPD()) {
            max = path.getSensorTarget().target;
            min = max * .95;
        } else {
            min = path.getSensorTarget().target;
            max = min * 1.05;
        }
        if (mPath.getSensorTarget().sensor == SensorTarget.Sensor.LeftIRPD) {
            mDrive.mLeftSensor.setLimitsVoltage(min, max);
        } else if (mPath.getSensorTarget().sensor == SensorTarget.Sensor.RearIRPD) {
            mDrive.mRearSensor.setLimitsVoltage(min, max);
        } else if (mPath.getSensorTarget().sensor == SensorTarget.Sensor.RightIRPD) {
            mDrive.mRearSensor.setLimitsVoltage(min, max);
        }
    }

    @Override
    public boolean isFinished() {

        return mIsFinished || (Timer.getFPGATimestamp() - Constants.Robot_Auton_Start_Time) >= 15;
    }

    @Override
    public void update() {
        mIsFinished = mDrive.updateAutonPath();
    }

    @Override
    public void done() {    }

    @Override
    public void start() {
        mDrive.setAutonPath(mPath, Timer.getFPGATimestamp());
        mStartTime = Timer.getFPGATimestamp();
    }
}

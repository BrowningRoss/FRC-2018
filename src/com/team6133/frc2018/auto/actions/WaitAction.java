package com.team6133.frc2018.auto.actions;

import com.team6133.frc2018.subsystems.Drive;
import edu.wpi.first.wpilibj.Timer;

/**
 * Action to wait for a given amount of time To use this Action, call
 * runAction(new WaitAction(your_time))
 */
public class WaitAction implements Action {

    private double mTimeToWait;
    private double mStartTime;
    private Drive mDrive = Drive.getInstance();

    public WaitAction(double timeToWait) {
        mTimeToWait = timeToWait;
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - mStartTime >= mTimeToWait;
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
        mDrive.stop();
    }
}
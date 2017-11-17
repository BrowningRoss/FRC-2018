package com.team6133.frc2018.auto.actions;

import com.team6133.frc2018.subsystems.MotorGearGrabber;
import com.team6133.frc2018.subsystems.MotorGearGrabber.WantedState;

/**
 * Action for scoring a gear
 * 
 * @see Action
 * @see RunOnceAction
 */
public class ScoreGearAction extends RunOnceAction {

    @Override
    public void runOnce() {
        MotorGearGrabber.getInstance().setWantedState(WantedState.SCORE);
    }
}
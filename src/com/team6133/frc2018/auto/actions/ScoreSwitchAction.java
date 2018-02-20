package com.team6133.frc2018.auto.actions;

import com.team6133.frc2018.subsystems.Intake;

public class ScoreSwitchAction extends RunOnceAction {
    @Override
    public void runOnce() {
        Intake.getInstance().setWantsExhaust();
        Intake.getInstance().setWantedState(Intake.WantedState.SCORE_SWITCH);
    }
}

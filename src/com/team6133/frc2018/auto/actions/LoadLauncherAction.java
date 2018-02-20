package com.team6133.frc2018.auto.actions;

import com.team6133.frc2018.subsystems.Intake;

public class LoadLauncherAction extends RunOnceAction {
    @Override
    public void runOnce() {
        Intake.getInstance().setWantedState(Intake.WantedState.LOAD_SHOOTER);
    }
}

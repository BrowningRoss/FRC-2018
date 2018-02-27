package com.team6133.frc2018.auto.actions;

import com.team6133.frc2018.subsystems.Launcher;

public class PrepLaunchScaleAction extends RunOnceAction {
    @Override
    public void runOnce() {
        Launcher.getInstance().setWantedState(Launcher.WantedState.ALIGN);
    }
}

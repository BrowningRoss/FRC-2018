package com.team6133.frc2018.auto.actions;

import com.team6133.frc2018.Constants;
import com.team6133.frc2018.subsystems.Launcher;
import edu.wpi.first.wpilibj.Timer;

public class PrepLaunchScaleAction extends RunOnceAction {
    @Override
    public void runOnce() {
        if (Timer.getFPGATimestamp() - Constants.Robot_Auton_Start_Time <= 15)
            Launcher.getInstance().setWantedState(Launcher.WantedState.ALIGN);
    }
}

package com.team6133.frc2018.auto.modes;

import com.team6133.frc2018.auto.AutoModeBase;
import com.team6133.frc2018.auto.AutoModeEndedException;
import com.team6133.frc2018.auto.AutonPathSettings;
import com.team6133.frc2018.auto.actions.DrivePathAction;
import com.team6133.frc2018.auto.actions.ResetStartingPoseAction;
import com.team6133.frc2018.auto.actions.ScoreSwitchAction;
import com.team6133.frc2018.auto.actions.WaitAction;
import com.team6133.lib.util.SensorTarget;
import com.team6133.lib.util.math.Rotation2d;

public class Option_StartCenter_EndScoreSwitchRight extends AutoModeBase {
    AutonPathSettings path = new AutonPathSettings(180, 36, 100, new SensorTarget(SensorTarget.Sensor.Ultra, 100, false), .5);

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Attempting to Score Right Switch");
        runAction(new ResetStartingPoseAction(Rotation2d.fromDegrees(180)));
        runAction(new DrivePathAction(AutonPathSettings.START_LSWITCH_160_END_SCORE_SWITCH));
        runAction(new ScoreSwitchAction());
        runAction(new WaitAction(15));
    }
}

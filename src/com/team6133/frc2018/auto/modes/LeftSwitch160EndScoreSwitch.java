package com.team6133.frc2018.auto.modes;

import com.team6133.frc2018.auto.AutoModeBase;
import com.team6133.frc2018.auto.AutoModeEndedException;
import com.team6133.frc2018.auto.AutonPathSettings;
import com.team6133.frc2018.auto.actions.DrivePathAction;
import com.team6133.frc2018.auto.actions.ResetStartingPoseAction;
import com.team6133.frc2018.auto.actions.ScoreSwitchAction;
import com.team6133.lib.util.math.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

public class LeftSwitch160EndScoreSwitch extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        double start = Timer.getFPGATimestamp();
        runAction(new DrivePathAction(AutonPathSettings.START_LSWITCH_160_END_SCORE_SWITCH));
        runAction(new ScoreSwitchAction());
        System.out.println("Score Switch Time:\t" + (Timer.getFPGATimestamp() - start));
    }
}

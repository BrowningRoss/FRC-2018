package com.team6133.frc2018.auto.modes;

import com.team6133.frc2018.auto.AutoModeBase;
import com.team6133.frc2018.auto.AutoModeEndedException;
import com.team6133.frc2018.auto.AutonPathSettings;
import com.team6133.frc2018.auto.actions.DrivePathAction;
import com.team6133.frc2018.auto.actions.ResetStartingPoseAction;
import com.team6133.frc2018.auto.actions.WaitAction;
import com.team6133.lib.util.math.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

public class StartLeftEndLeftSwitch160Mode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        double start = Timer.getFPGATimestamp();
        runAction(new ResetStartingPoseAction(Rotation2d.fromDegrees(90)));
        runAction(new DrivePathAction(AutonPathSettings.START_LEFT_END_LSWITCH_160));
        System.out.println("Path Left@160 Time:\t" + (Timer.getFPGATimestamp() - start));
    }
}

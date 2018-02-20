package com.team6133.frc2018.auto.modes;

import com.team6133.frc2018.auto.AutoModeBase;
import com.team6133.frc2018.auto.AutoModeEndedException;
import com.team6133.frc2018.auto.AutonPathSettings;
import com.team6133.frc2018.auto.actions.DrivePathAction;
import com.team6133.frc2018.auto.actions.ResetStartingPoseAction;
import com.team6133.frc2018.auto.actions.ScoreSwitchAction;
import com.team6133.frc2018.auto.actions.WaitAction;
import com.team6133.lib.util.math.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

public class Option_StartLeft_EndScoreLeftSwitch extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Attempting to Score Left Switch");
        double start = Timer.getFPGATimestamp();
        runAction(new ResetStartingPoseAction(Rotation2d.fromDegrees(FACE_LEFT)));
        runAction(new DrivePathAction(AutonPathSettings.START_LEFT_END_LSWITCH_160));
        System.out.println("Path Left@160 Time:\t" + (Timer.getFPGATimestamp() - start));

        runAction(new DrivePathAction(AutonPathSettings.START_LSWITCH_160_END_SCORE_SWITCH));
        //runAction(new ScoreSwitchAction());
        System.out.println("Score Switch Time:\t" + (Timer.getFPGATimestamp() - start));

        runAction(new WaitAction(15));
    }
}

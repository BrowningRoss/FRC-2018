package com.team6133.frc2018.auto.modes;

import com.team6133.frc2018.Constants;
import com.team6133.frc2018.auto.AutoModeBase;
import com.team6133.frc2018.auto.AutoModeEndedException;
import com.team6133.frc2018.auto.AutonPathSettings;
import com.team6133.frc2018.auto.actions.*;
import com.team6133.lib.util.SensorTarget;
import com.team6133.lib.util.math.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

import java.util.Arrays;

public class Option_StartLeft_EndScoreLeftSwitch extends AutoModeBase {
    AutonPathSettings path1a = new AutonPathSettings(FACE_LEFT, 0,160, new SensorTarget(SensorTarget.Sensor.LeftIRPD, 1.32, false), 0.25, .66);
    AutonPathSettings path1b = new AutonPathSettings(FACE_LEFT, 0,160, new SensorTarget(SensorTarget.Sensor.LeftIRPD, 1.32, true), 0.5, .85);
    AutonPathSettings path2 = new AutonPathSettings(FACE_LEFT, 12,0, new SensorTarget(SensorTarget.Sensor.Ultra, Constants.SWITCH_SIDE_DISTANCE_INCHES, false) , .8, .5);

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Option_StartLeft_EndScoreLeftSwitch()");
        double start = Timer.getFPGATimestamp();
        runAction(new ResetStartingPoseAction(Rotation2d.fromDegrees(FACE_LEFT)));
        runAction(new DrivePathAction(path1a));
        runAction(new DrivePathAction(path1b));
        System.out.println("Path 1/2 Time:\t" + (Timer.getFPGATimestamp() - start));

        runAction(new DrivePathAction(path2));
        System.out.println("Path 2/2 Time:\t" + (Timer.getFPGATimestamp() - start));

        runAction(new PrepLaunchSwitchAction());
        runAction(new WaitAction(.1));
        runAction(new LaunchSwitchAction());
        System.out.println("Score Switch Time:\t" + (Timer.getFPGATimestamp() - start));

        runAction(new WaitAction(15));
    }
}

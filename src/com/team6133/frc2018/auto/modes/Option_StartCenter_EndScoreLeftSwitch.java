package com.team6133.frc2018.auto.modes;

import com.team6133.frc2018.auto.AutoModeBase;
import com.team6133.frc2018.auto.AutoModeEndedException;
import com.team6133.frc2018.auto.AutonPathSettings;
import com.team6133.frc2018.auto.actions.*;
import com.team6133.lib.util.SensorTarget;
import com.team6133.lib.util.math.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

import java.util.Arrays;

public class Option_StartCenter_EndScoreLeftSwitch extends AutoModeBase {
    AutonPathSettings path0 = new AutonPathSettings(FACE_LEFT, -55, 0, new SensorTarget(SensorTarget.Sensor.Time, 1,true), 1.73, .5);
    AutonPathSettings path1 = new AutonPathSettings(FACE_LEFT, -55, 0, new SensorTarget(SensorTarget.Sensor.RightIRPD, 1.61, true), .75, .5);
    AutonPathSettings path2 = new AutonPathSettings(FACE_LEFT, -55, 0, new SensorTarget(SensorTarget.Sensor.RightIRPD, 1.61, true), .75, .5);
    AutonPathSettings path3 = new AutonPathSettings(FACE_LEFT, 0, 45, new SensorTarget(SensorTarget.Sensor.LeftIRPD, 2.25, true), .7, .5);
    AutonPathSettings path4 = new AutonPathSettings(0, 0, 55, new SensorTarget(SensorTarget.Sensor.Time, 2.25, false), 3, .25);

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Option_StartCenter_EndScoreLeftSwitch()");
        double start = Timer.getFPGATimestamp();
        runAction(new ResetStartingPoseAction(Rotation2d.fromDegrees(FACE_LEFT)));
        runAction(new DrivePathAction(path1));
        System.out.println("Path 1/4 Time:\t" + (Timer.getFPGATimestamp() - start));

        runAction(new DrivePathAction(path2));
        System.out.println("Path 2/4 Time:\t" + (Timer.getFPGATimestamp() - start));

        runAction(new DrivePathAction(path3));
        System.out.println("Path 3/4 Time:\t" + (Timer.getFPGATimestamp() - start));

        runAction(new DrivePathAction(path4));
        System.out.println("Path 4/4 Time:\t" + (Timer.getFPGATimestamp() - start));

        runAction(new PrepLaunchSwitchAction());
        runAction(new WaitAction(.1));
        runAction(new LaunchSwitchAction());
        System.out.println("Score Switch Time:\t" + (Timer.getFPGATimestamp() - start));

        runAction(new WaitAction(15));
    }
}

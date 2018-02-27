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

public class Option_StartLeft_EndScoreRightSwitch extends AutoModeBase {
    AutonPathSettings path1 = new AutonPathSettings(FACE_LEFT, 36, 48, new SensorTarget(SensorTarget.Sensor.LeftIRPD, 48, true), .5);
    AutonPathSettings path2 = new AutonPathSettings(0, 100, 0, new SensorTarget(SensorTarget.Sensor.RightIRPD, 28, false), 2);
    AutonPathSettings path3 = new AutonPathSettings(FACE_RIGHT, 0, 12, new SensorTarget(SensorTarget.Sensor.RightIRPD, 150, true), 1);
    AutonPathSettings path4 = new AutonPathSettings(FACE_RIGHT, -12, 0, new SensorTarget(SensorTarget.Sensor.Ultra, Constants.SWITCH_SIDE_DISTANCE_INCHES, false), .8);

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Option_StartLeft_EndScoreRightSwitch()");
        double start = Timer.getFPGATimestamp();
        runAction(new ResetStartingPoseAction(Rotation2d.fromDegrees(FACE_LEFT)));
        runAction(new DrivePathAction(path1));
        System.out.println("Path 1/4 Time:\t" + (Timer.getFPGATimestamp() - start));

        runAction(new DrivePathAction(path2));
        System.out.println("Path 2/4 Time:\t" + (Timer.getFPGATimestamp() - start));

        runAction(new DrivePathAction(path3));
        System.out.println("Path 3/4 Time:\t" + (Timer.getFPGATimestamp() - start));

        runAction( new ParallelAction(Arrays.asList(new Action[] {
                new DrivePathAction(path4),
                new PrepLaunchSwitchAction(),
        })));
        System.out.println("Path 4/4 Time:\t" + (Timer.getFPGATimestamp() - start));
        runAction(new LaunchSwitchAction());
        System.out.println("Score Switch Time:\t" + (Timer.getFPGATimestamp() - start));

        runAction(new WaitAction(15));
    }
}

package com.team6133.frc2018.auto.modes;

import com.team6133.frc2018.auto.AutoModeBase;
import com.team6133.frc2018.auto.AutoModeEndedException;
import com.team6133.frc2018.auto.AutonPathSettings;
import com.team6133.frc2018.auto.actions.*;
import com.team6133.lib.util.SensorTarget;
import com.team6133.lib.util.math.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

import java.util.Arrays;

public class Option_StartRight_EndScoreScale extends AutoModeBase {
    AutonPathSettings path1 = new AutonPathSettings(FACE_LEFT, 0,216, new SensorTarget(SensorTarget.Sensor.LeftIRPD, 1.2, true), .5, .66);
    AutonPathSettings path2 = new AutonPathSettings(FACE_RIGHT, 0, 200, new SensorTarget(SensorTarget.Sensor.RearIRPD, 1.55, false), 1.5, .66);
    AutonPathSettings path3 = new AutonPathSettings(FACE_RIGHT, 12, 0, SensorTarget.ultraScaleLaunchSetpoint, .05, .5);

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Option_StartRight_EndScoreScale()");
        double start = Timer.getFPGATimestamp();
        runAction(new ResetStartingPoseAction(Rotation2d.fromDegrees(FACE_LEFT)));
        runAction(new DrivePathAction(path1));
        System.out.println("Path 1/3 Time:\t"+(Timer.getFPGATimestamp() - start));

        runAction(new DrivePathAction(path2));
        System.out.println("Path 2/3 Time:\t" + (Timer.getFPGATimestamp() - start));

        runAction(new DrivePathAction(path3));
        System.out.println("Path 3/3 Time:\t" + (Timer.getFPGATimestamp() - start));

        runAction(new PrepLaunchScaleAction());
        runAction(new LaunchScaleAction());
        System.out.println("Score Scale Time:\t"+(Timer.getFPGATimestamp() - start));
        runAction(new WaitAction(15));
    }
}

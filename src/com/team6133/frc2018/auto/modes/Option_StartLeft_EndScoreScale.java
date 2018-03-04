package com.team6133.frc2018.auto.modes;

import com.team6133.frc2018.auto.AutoModeBase;
import com.team6133.frc2018.auto.AutoModeEndedException;
import com.team6133.frc2018.auto.AutonPathSettings;
import com.team6133.frc2018.auto.actions.*;
import com.team6133.lib.util.SensorTarget;
import com.team6133.lib.util.math.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

import java.util.Arrays;

public class Option_StartLeft_EndScoreScale extends AutoModeBase {
    AutonPathSettings path1 = new AutonPathSettings(FACE_LEFT, 0,160, new SensorTarget(SensorTarget.Sensor.LeftIRPD, 150, true), .5);
    AutonPathSettings path2 = new AutonPathSettings(FACE_LEFT, -20, 164, new SensorTarget(SensorTarget.Sensor.Ultra, 10, false), .5);
    AutonPathSettings path3 = new AutonPathSettings(FACE_LEFT, 1, 0, SensorTarget.ultraScaleLaunchSetpoint, .05);

    @Override
    protected void routine() throws AutoModeEndedException {
    System.out.println("Option_StartLeft_EndScoreScale()");
    double start = Timer.getFPGATimestamp();
    runAction(new ResetStartingPoseAction(Rotation2d.fromDegrees(FACE_LEFT)));
    if ((Timer.getFPGATimestamp() - start < 15)) {
        runAction(new DrivePathAction(path1));
        System.out.println("Path 1/3 Time:\t" + (Timer.getFPGATimestamp() - start));
    } else
        return;
    if ((Timer.getFPGATimestamp() - start < 15)) {
        runAction(new DrivePathAction(path2));
        runAction(new PrepLaunchScaleAction());
        System.out.println("Path 2/3 Time:\t" + (Timer.getFPGATimestamp() - start));
    } else {
        System.out.println("Ended path2");
        return;
    }

    if ((Timer.getFPGATimestamp() - start < 15)) {
        runAction(new DrivePathAction(path3));
        System.out.println("Path 3/3 Time:\t" + (Timer.getFPGATimestamp() - start));
    } else
        return;
    if ((Timer.getFPGATimestamp() - start < 15)) {
        runAction(new LaunchScaleAction());
        System.out.println("Score Scale Time:\t" + (Timer.getFPGATimestamp() - start));
    } else
        return;
    if ((Timer.getFPGATimestamp() - start < 15)) {
        runAction(new WaitAction(15));
    } else
        return;
    }
}

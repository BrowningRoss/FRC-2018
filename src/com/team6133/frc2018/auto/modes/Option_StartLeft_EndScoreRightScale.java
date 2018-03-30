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

public class Option_StartLeft_EndScoreRightScale extends AutoModeBase {
    AutonPathSettings path1a = new AutonPathSettings(FACE_LEFT, 0,1, new SensorTarget(SensorTarget.Sensor.LeftIRPD, 1.2, false), .15, .66);
    AutonPathSettings path1b = new AutonPathSettings(FACE_LEFT, 0,1, new SensorTarget(SensorTarget.Sensor.LeftIRPD, 1.2, true), .25, .85);
    AutonPathSettings path1c = new AutonPathSettings(FACE_LEFT, 0,1, new SensorTarget(SensorTarget.Sensor.LeftIRPD, 1.2, true), .35, .85);
    AutonPathSettings path2a = new AutonPathSettings(FACE_LEFT, 1, 0, new SensorTarget(SensorTarget.Sensor.RearIRPD, 1.55, false), 1, .5);
    AutonPathSettings path3a = new AutonPathSettings(FACE_RIGHT, 0, 1, new SensorTarget(SensorTarget.Sensor.RearIRPD, 1.55, false), 1.5, .66);
    AutonPathSettings path3b = new AutonPathSettings(FACE_RIGHT, 0, 1, new SensorTarget(SensorTarget.Sensor.RearIRPD, 1.55, false), .5, .66);
    //AutonPathSettings path4 = new AutonPathSettings(FACE_RIGHT, 1,0, new SensorTarget(SensorTarget.Sensor.Ultra, Constants.kLaunchProxSetpoint, false), 0.5, .4);
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Option_StartLeft_EndScoreScale()");
        double start = Timer.getFPGATimestamp();
        runAction(new ResetStartingPoseAction(Rotation2d.fromDegrees(FACE_LEFT)));
        runAction(new DrivePathAction(path1a));
        runAction(new DrivePathAction(path1b));
        runAction(new DrivePathAction(path1c));
        System.out.println("Path 1/3 Time:\t"+(Timer.getFPGATimestamp() - start));

        runAction(new DrivePathAction(path2a));
        System.out.println("Path 2/3 Time:\t"+(Timer.getFPGATimestamp() - start));

        runAction(new DrivePathAction(path3a));
        runAction(new DrivePathAction(path3b));
        System.out.println("Path 3/3 Time:\t" + (Timer.getFPGATimestamp() - start));

        //runAction(new DrivePathAction(path4));
        //System.out.println("Path 4/4 Time:\t" + (Timer.getFPGATimestamp() - start));

        runAction(new PrepLaunchScaleAction());
        runAction(new WaitAction(.33));
        runAction(new LaunchScaleAction());
        System.out.println("Score Scale Time:\t"+(Timer.getFPGATimestamp() - start));
        runAction(new WaitAction(15));
    }
}

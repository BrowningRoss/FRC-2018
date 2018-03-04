package com.team6133.frc2018.auto.modes;

import com.team6133.frc2018.auto.AutoModeBase;
import com.team6133.frc2018.auto.AutoModeEndedException;
import com.team6133.frc2018.auto.AutonPathSettings;
import com.team6133.frc2018.auto.actions.*;
import com.team6133.lib.util.SensorTarget;
import com.team6133.lib.util.math.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

import java.util.Arrays;

public class Option_StartLeft_EndScoreRightScale extends AutoModeBase {
    AutonPathSettings path1 = new AutonPathSettings(FACE_LEFT, 0,216, new SensorTarget(SensorTarget.Sensor.LeftIRPD, 216, true), .5);
    AutonPathSettings path2 = new AutonPathSettings(0, 100, 0, new SensorTarget(SensorTarget.Sensor.RightIRPD, 32, false), .5);
    AutonPathSettings path3 = new AutonPathSettings(0, 0, 1, new SensorTarget(SensorTarget.Sensor.LeftIRPD, 72, false), .25);
    AutonPathSettings path4 = new AutonPathSettings(115, 0, 0, new SensorTarget(SensorTarget.Sensor.Gyro, 115, false), 0.1);

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Option_StartLeft_EndScoreScale()");
        double start = Timer.getFPGATimestamp();
        runAction(new ResetStartingPoseAction(Rotation2d.fromDegrees(FACE_LEFT)));
        runAction(new DrivePathAction(path1));
        System.out.println("Path 1/4 Time:\t"+(Timer.getFPGATimestamp() - start));

        runAction(new DrivePathAction(path2));
        System.out.println("Path 2/4 Time:\t"+(Timer.getFPGATimestamp() - start));

        runAction(new ParallelAction(Arrays.asList(new Action[] {
                new DrivePathAction(path3),
                new PrepLaunchScaleAction(),
        })));
        System.out.println("Path 3/4 Time:\t" + (Timer.getFPGATimestamp() - start));

        runAction(new DrivePathAction(path4));
        System.out.println("Path 4/4 Time:\t" + (Timer.getFPGATimestamp() - start));

        runAction(new LaunchScaleAction());
        System.out.println("Score Scale Time:\t"+(Timer.getFPGATimestamp() - start));
        runAction(new WaitAction(15));
    }

    @Override
    public void stop() {
        super.stop();
    }
}

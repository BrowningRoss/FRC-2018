package com.team6133.frc2018.auto.modes;

import com.team6133.frc2018.auto.AutoModeBase;
import com.team6133.frc2018.auto.AutoModeEndedException;
import com.team6133.frc2018.auto.AutonPathSettings;
import com.team6133.frc2018.auto.actions.DrivePathAction;
import com.team6133.frc2018.auto.actions.ResetStartingPoseAction;
import com.team6133.frc2018.auto.actions.WaitAction;
import com.team6133.lib.util.SensorTarget;
import com.team6133.lib.util.math.Rotation2d;

public class TestPathMode extends AutoModeBase {
    AutonPathSettings path1a = new AutonPathSettings(FACE_LEFT, 0,216, new SensorTarget(SensorTarget.Sensor.LeftIRPD, 1.2, false), .25, .66);
    AutonPathSettings path1b = new AutonPathSettings(FACE_LEFT, 0,216, new SensorTarget(SensorTarget.Sensor.LeftIRPD, 1.2, true), .5, .85);

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new ResetStartingPoseAction(Rotation2d.fromDegrees(FACE_LEFT)));
        //System.out.println("Rotating to 0deg (face forward)");
        //runAction(new DrivePathAction(rotate1));
        //runAction(new WaitAction(1));

        //System.out.println("Rotating to " + FACE_LEFT + "deg (face left)");
        //runAction(new DrivePathAction(rotate2));
        //runAction(new WaitAction(1));
        runAction(new DrivePathAction(path1a));
        runAction(new DrivePathAction(path1b));
        //runAction(new DrivePathAction(path1c));
        //runAction(new DrivePathAction(path1));
        runAction(new WaitAction(1));

        System.out.println("Test done");
    }
}

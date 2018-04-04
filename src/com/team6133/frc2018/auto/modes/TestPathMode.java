package com.team6133.frc2018.auto.modes;

import com.team6133.frc2018.auto.AutoModeBase;
import com.team6133.frc2018.auto.AutoModeEndedException;
import com.team6133.frc2018.auto.AutonPathSettings;
import com.team6133.frc2018.auto.actions.*;
import com.team6133.lib.util.SensorTarget;
import com.team6133.lib.util.math.Rotation2d;

public class TestPathMode extends AutoModeBase {
    AutonPathSettings path3 = new AutonPathSettings(FACE_LEFT, 0, 55, new SensorTarget(SensorTarget.Sensor.LeftIRPD, 2.25, true), .7, .5);
    AutonPathSettings path4 = new AutonPathSettings(0, 0, 55, new SensorTarget(SensorTarget.Sensor.RearIRPD, 2.25, true), 3, .4);

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new ResetStartingPoseAction(Rotation2d.fromDegrees(FACE_LEFT)));
        //System.out.println("Rotating to 0deg (face forward)");
        //runAction(new DrivePathAction(rotate1));
        //runAction(new WaitAction(1));

        //System.out.println("Rotating to " + FACE_LEFT + "deg (face left)");
        //runAction(new DrivePathAction(rotate2));
        //runAction(new WaitAction(1));
        runAction(new DrivePathAction(path3));
        runAction(new DrivePathAction(path4));
        runAction(new PrepLaunchSwitchAction());
        runAction(new WaitAction(.1));
        runAction(new LaunchSwitchAction());
        //runAction(new DrivePathAction(path1));
        runAction(new WaitAction(1));

        System.out.println("Test done");
    }
}

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
    double dx = -15, dy = 164, target = 10;
    double dx2 = 1, dy2 = 0, target2 = 18;
    AutonPathSettings rotate1 = new AutonPathSettings(0, 0, 0, new SensorTarget(SensorTarget.Sensor.Gyro, 0, false),.1);
    AutonPathSettings rotate2 = new AutonPathSettings(FACE_LEFT, 0, 0, new SensorTarget(SensorTarget.Sensor.Gyro, FACE_LEFT, false),.1);
    AutonPathSettings path1 = new AutonPathSettings(FACE_LEFT, dx, dy, new SensorTarget(SensorTarget.Sensor.Ultra, target, false), .5);
    AutonPathSettings path2 = new AutonPathSettings(FACE_LEFT, dx2, dx2, new SensorTarget(SensorTarget.Sensor.Ultra, target2, false), .05);

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new ResetStartingPoseAction(Rotation2d.fromDegrees(FACE_LEFT)));
        System.out.println("Rotating to 0deg (face forward)");
        runAction(new DrivePathAction(rotate1));
        runAction(new WaitAction(1));

        System.out.println("Rotating to " + FACE_LEFT + "deg (face left)");
        runAction(new DrivePathAction(rotate2));
        runAction(new WaitAction(1));

        System.out.println("Driving dx: " + dx + ", dy: " + dy + ", target inches: " + target);
        runAction(new DrivePathAction(path1));
        runAction(new WaitAction(1));

        System.out.println("Driving dx: " + dx2 + ", dy: " + dy2 + ", target inches: " + target2);
        runAction(new DrivePathAction(path2));
        runAction(new WaitAction(1));

        System.out.println("Test done");
    }
}

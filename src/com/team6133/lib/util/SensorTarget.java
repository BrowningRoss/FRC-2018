package com.team6133.lib.util;

import com.team6133.frc2018.Constants;

public class SensorTarget {
    public double target;
    public Sensor sensor;
    public boolean invert;

    public enum Sensor {
        LeftIRPD, RightIRPD, Ultra, Gyro
    }

    public SensorTarget(Sensor s, double t, boolean i) {
        sensor = s;
        target = t;
        invert = i;
    }

    public static final SensorTarget ultraScaleLaunchSetpoint = new SensorTarget(Sensor.Ultra, Constants.kLaunchProxSetpoint, false);
}


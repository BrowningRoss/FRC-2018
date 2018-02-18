package com.team6133.lib.util;

import com.team6133.frc2018.Constants;

public class SensorTarget {
    public double target;
    public Sensor sensor;
    public boolean invert;

    public enum Sensor {
        LeftIRPD, RightIRPD, Ultra
    }

    public SensorTarget(Sensor s, double t, boolean i) {
        sensor = s;
        target = t;
        invert = i;
    }

    public static final SensorTarget leftIRPD = new SensorTarget(Sensor.LeftIRPD, 0, false);
    public static final SensorTarget leftIRPDinvert = new SensorTarget(Sensor.LeftIRPD, 0, true);
    public static final SensorTarget rightIRPD = new SensorTarget(Sensor.RightIRPD, 0, false);
    public static final SensorTarget rightIRPDinvert = new SensorTarget(Sensor.RightIRPD, 0, true);
    public static final SensorTarget ultraScale = new SensorTarget(Sensor.Ultra, Constants.kLaunchProxSetpoint, false);
}


package com.team6133.frc2018.auto;

import com.team6133.frc2018.Constants;
import com.team6133.lib.util.SensorTarget;
import com.team6133.lib.util.math.Rotation2d;

public class AutonPathSettings {
    private final Rotation2d mHeading;
    private final Rotation2d mOrientation;
    private final SensorTarget mSensorTarget;
    private final boolean mInvertIRPD;
    private final double mTimeout;

    public AutonPathSettings(double heading, double orientation, SensorTarget sensor_target, double timeout) {
        mHeading = Rotation2d.fromDegrees(heading);
        mOrientation = Rotation2d.fromDegrees(orientation);
        mSensorTarget = sensor_target;
        mInvertIRPD = sensor_target.invert;
        mTimeout = timeout;
    }

    public Rotation2d getHeading() {return mHeading;}

    public Rotation2d getOrientation() {return mOrientation;}

    public SensorTarget getSensorTarget() {return mSensorTarget;}

    public boolean getInvertIRPD() {return mInvertIRPD;}

    public double getTimeout() { return mTimeout;}

    public static final AutonPathSettings START_LEFT_END_LSWITCH_FAR =
            new AutonPathSettings(0, 90, SensorTarget.leftIRPD, 0.5);

    public static final AutonPathSettings START_LSWITCH_FAR_END_LSCALE =
            new AutonPathSettings(6.953, 90, new SensorTarget(SensorTarget.Sensor.Ultra, 10, false), 0.5);

    public static final AutonPathSettings START_LSWITCH_FAR_END_SCORE_SWITCH =
            new AutonPathSettings(90, -90, new SensorTarget(SensorTarget.Sensor.Ultra, 4, false) , .8);

    public static final AutonPathSettings START_LSCALE_END_SCORE_SCALE =
            new AutonPathSettings(90, 90, SensorTarget.ultraScale, 0);

}

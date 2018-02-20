package com.team6133.frc2018.auto;

import com.team6133.frc2018.Constants;
import com.team6133.lib.util.SensorTarget;
import com.team6133.lib.util.math.Rotation2d;

public class AutonPathSettings {
    private final Rotation2d mHeading;
    private final double dX;
    private final double dY;
    private final SensorTarget mSensorTarget;
    private final boolean mInvertIRPD;
    private final double mTimeout;
    private final double mMagnitudeX, mMagnitudeY;

    protected static double FACE_LEFT = -90;
    protected static double FACE_RIGHT = 90;
    protected static double FACE_FORWARD = 0;

    public AutonPathSettings(double heading, double dx, double dy, SensorTarget sensor_target, double timeout) {
        mHeading = Rotation2d.fromDegrees(heading);
        dX = dx;
        dY = dy;
        mSensorTarget = sensor_target;
        mInvertIRPD = sensor_target.invert;
        mTimeout = timeout;
        double angle = calculateAngle();
        if (heading == FACE_LEFT) {
            mMagnitudeX = dX > 0 ? -1 * Math.abs(Math.sin(angle)) : Math.abs(Math.sin(angle));
            mMagnitudeY = dY > 0 ? -1 * Math.abs(Math.cos(angle)) : Math.abs(Math.cos(angle));
        } else {
            mMagnitudeX = dX < 0 ? -1 * Math.abs(Math.sin(angle)) : Math.abs(Math.sin(angle));
            mMagnitudeY = dY < 0 ? -1 * Math.abs(Math.cos(angle)) : Math.abs(Math.cos(angle));
        }
    }

    private double calculateAngle() {
        if (dY != 0) {
            return Math.atan(dX / dY);
        } else {
            return Math.PI / 2;
        }
    }

    public double getMagnitudeX() {
        if (dY == 0) {
            return mMagnitudeX * .5;
        }
        double ratio = dX / dY;

        return mMagnitudeX;
    }

    public double getMagnitudeY() {
        if (dX == 0) {
            return mMagnitudeY * .5;
        }
        return mMagnitudeY;
    }

    public Rotation2d getHeading() {return mHeading;}

    public SensorTarget getSensorTarget() {return mSensorTarget;}

    public boolean getInvertIRPD() {return mInvertIRPD;}

    public double getTimeout() { return mTimeout;}

    public static final AutonPathSettings START_LEFT_END_LSWITCH_160 =
            new AutonPathSettings(FACE_LEFT, 0,160, new SensorTarget(SensorTarget.Sensor.LeftIRPD, 50, true), 0.5);

    public static final AutonPathSettings START_LSWITCH_FAR_END_LSCALE =
            new AutonPathSettings(FACE_LEFT,-20, 164, new SensorTarget(SensorTarget.Sensor.Ultra, 10, false), 0.5);

    public static final AutonPathSettings START_LSWITCH_160_END_SCORE_SWITCH =
            new AutonPathSettings(FACE_RIGHT, 12,0, new SensorTarget(SensorTarget.Sensor.Ultra, 8, false) , .8);

    public static final AutonPathSettings START_LSCALE_160_SCORE_SCALE =
            new AutonPathSettings(FACE_LEFT, 12,0, SensorTarget.ultraScaleLaunchSetpoint, 0);

}

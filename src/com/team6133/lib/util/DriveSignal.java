package com.team6133.lib.util;

/**
 * A drivetrain command consisting of the x speed, y speed settings and whether
 * the twist (z-axis).
 */
public class DriveSignal {
    public static DriveSignal NEUTRAL = new DriveSignal(0, 0, 0);
    public static DriveSignal FORWARD = new DriveSignal(0, 0, 0);
    protected double mXspeed;
    protected double mYspeed;
    protected double mTwist;

    public DriveSignal(double xSpeed, double ySpeed, double twist) {
        mXspeed = xSpeed;
        mYspeed = ySpeed;
        mTwist = twist;
    }

    public double getX() {
        return mXspeed;
    }

    public double getY() {
        return mYspeed;
    }

    public double getTwist() {
        return mTwist;
    }

    @Override
    public String toString() {
        return "X: " + mXspeed + ", Y: " + mYspeed + ", Twist: " + mTwist;
    }
}
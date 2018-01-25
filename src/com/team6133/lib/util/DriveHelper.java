package com.team6133.lib.util;

public class DriveHelper {

    private static final double kThrottleDeadband_X = 0.15;
    private static final double kThrottleDeadband_Y = 0.10;
    private static final double kThrottleDeadband_Twist = 0.25;

    private static final double kMin_X = 0.02;
    private static final double kMin_Y = 0.02;
    private static final double kMin_Twist = 0.02;

    private static final double kMax_X =  1.0;
    private static final double kMax_Y =  1.0;
    private static final double kMax_Twist =  0.66;

    private static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kThrottle_X = new InterpolatingTreeMap<>();
    private static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kThrottle_Y = new InterpolatingTreeMap<>();
    private static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kThrottle_Twist = new InterpolatingTreeMap<>();

    private static final double[][] kThrottleValues_X = {
            {-1.0                , -kMax_X},
            {-0.75               , -0.5 * kMax_X},
            {-kThrottleDeadband_X, -kMin_X},
            {kThrottleDeadband_X, kMin_X},
            {0.75               , 0.5 * kMax_X},
            {1.0                , kMax_X}
    };
    private static final double[][] kThrottleValues_Y = {
            {-1.0                , -kMax_Y},
            {-0.75               , -0.5 * kMax_Y},
            {-kThrottleDeadband_Y, -kMin_Y},
            {kThrottleDeadband_Y, kMin_Y},
            {0.75               , 0.5 * kMax_Y},
            {1.0                , kMax_Y}
    };
    private static final double[][] kThrottleValues_Twist = {
            {-1.0                    , -kMax_Twist},
            {-0.75                   , -0.5 * kMax_Twist},
            {-kThrottleDeadband_Twist, -kMin_Twist},
            {kThrottleDeadband_Twist, kMin_Twist},
            {0.75                   , 0.5 * kMax_Twist},
            {1.0                    , kMax_Twist}
    };

    static {
        for (double[] pair : kThrottleValues_X) {
            kThrottle_X.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }
        for (double[] pair : kThrottleValues_Y) {
            kThrottle_Y.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }
        for (double[] pair : kThrottleValues_Twist) {
            kThrottle_Twist.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }
    }

    public DriveSignal mecDrive(double x, double y, double twist) {

        x = handleDeadband(x, kThrottleDeadband_X);
        y = handleDeadband(y, kThrottleDeadband_Y);
        twist = handleDeadband(twist, kThrottleDeadband_Twist);

        final double adjusted_x, adjusted_y, adjusted_twist;
        adjusted_x = x == 0.0 ? 0.0 : kThrottle_X.getInterpolated(new InterpolatingDouble(x)).value;
        adjusted_y = y == 0.0 ? 0.0 : kThrottle_Y.getInterpolated(new InterpolatingDouble(y)).value;
        adjusted_twist =  twist == 0.0 ? 0.0 : kThrottle_Twist.getInterpolated(new InterpolatingDouble(twist)).value;

        return new DriveSignal(-adjusted_x, adjusted_y, adjusted_twist);
    }

    public double handleDeadband(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }
}

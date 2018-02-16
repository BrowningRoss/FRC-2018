package com.team6133.lib.util.drivers;

import edu.wpi.first.wpilibj.Ultrasonic;

import java.util.LinkedList;

/**
 * Driver for a trigger/echo analog Ultrasonic Sensor (mainly to help smooth out noise).
 */
public class UltrasonicSensor extends Ultrasonic {
    private static final int kCacheSize = 5;
    private final double mScalingFactor = 1.0 / 148.0;
    private LinkedList<Double> cache;

    public UltrasonicSensor(int ping_port, int echo_port) {
        super(ping_port, echo_port, Unit.kInches);
        cache = new LinkedList<Double>();
        setAutomaticMode(true);
        cache.add(getRangeInches());
    }

    public void update() {
        cache.add(getRangeInches());
        if (cache.size() > kCacheSize)
            cache.removeFirst();
    }

    public double getAverageDistance() {
        double total = 0;
        for (Double d : cache) {
            total += d;
        }
        return total / cache.size();
    }

    public double getLatestDistance() {
        return cache.getLast();
    }
}
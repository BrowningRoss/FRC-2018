package com.team6133.frc2018.subsystems;

import com.team6133.frc2018.loops.Loop;
import com.team6133.frc2018.loops.Looper;
import com.team6133.lib.util.LatchedBoolean;

/**
 * Keeps track of the robot's connection to the driver station. If it
 * disconnects for more than 1 second, start blinking the LEDs.
 */
public class ConnectionMonitor extends Subsystem {

    public static double kConnectionTimeoutSec = 1.0;

    private static ConnectionMonitor mInstance = null;
    private double mLastPacketTime;
    private LatchedBoolean mJustReconnected;
    private LatchedBoolean mJustDisconnected;
    private LED mLED;
    ConnectionMonitor() {
        mLastPacketTime = 0.0;
        mJustReconnected = new LatchedBoolean();
        mJustDisconnected = new LatchedBoolean();
        mLED = LED.getInstance();
    }

    public static ConnectionMonitor getInstance() {
        if (mInstance == null) {
            mInstance = new ConnectionMonitor();
        }
        return mInstance;
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (ConnectionMonitor.this) {
                    mLastPacketTime = timestamp;
                }
            }

            @Override
            public void onLoop(double timestamp) {
            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }

    @Override
    public void outputToSmartDashboard() {
    }

    @Override
    public void stop() {

    }

    @Override
    public void zeroSensors() {

    }

    public synchronized void setLastPacketTime(double timestamp) {
        mLastPacketTime = timestamp;
    }
}
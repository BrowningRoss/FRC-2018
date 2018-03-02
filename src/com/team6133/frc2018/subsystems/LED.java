package com.team6133.frc2018.subsystems;

import com.team6133.frc2018.Constants;
import com.team6133.frc2018.loops.Loop;
import com.team6133.frc2018.loops.Looper;
import edu.wpi.first.wpilibj.DigitalOutput;

/**
 * The LED subsystem consists of three 5V LED strips, controlled by a 32bit Arduino.
 * When the Robot initializes, the LEDs will do an orange showcase.
 * When the Robot receives its FMS Alliance Color, the LEDs turn to match that color.
 * During a match, the LED strip is used for signaling to the human player.
 * The main things this subsystem has to do is signal the Arduino to handle state changes.
 *
 * @see Subsystem
 */
public class LED extends Subsystem {
    private static LED mInstance = null;
    private SystemState mSystemState = SystemState.PRE_MATCH;
    private WantedState mWantedState = WantedState.PRE_MATCH;
    private boolean mHasReset;
    private DigitalOutput mRED;
    private DigitalOutput mBLUE;
    private DigitalOutput mReset;

    private Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized (LED.this) {
                mSystemState = SystemState.PRE_MATCH;
                mWantedState = WantedState.PRE_MATCH;
                mRED.set(true);
                mBLUE.set(true);
                mReset.set(true);
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (LED.this) {
                SystemState newState;
                switch (mSystemState) {
                    case PRE_MATCH:
                        newState = handlePreMatch();
                        break;
                    case ALLIANCE_RED:
                        newState = handleRedAlliance();
                        break;
                    case ALLIANCE_BLUE:
                        newState = handleBlueAlliance();
                        break;
                    case SIGNALING:
                        newState = handleSignaling();
                        break;
                    default:
                        newState = handleReset();
                        break;
                }
                if (newState != mSystemState) {
                    System.out.println("LED state " + mSystemState + " to " + newState);
                    mSystemState = newState;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {}
    };
    private LED() {
        mBLUE = new DigitalOutput(Constants.kAllianceBlueLEDId);
        mBLUE.set(true);

        mRED = new DigitalOutput(Constants.kAllianceRedLEDId);
        mRED.set(true);

        mReset = new DigitalOutput(Constants.kResetArduinoId);
        mReset.set(true);
        mHasReset = false;
    }

    public static LED getInstance() {
        if (mInstance == null) {
            mInstance = new LED();
        }
        return mInstance;
    }

    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case PRE_MATCH:
                return SystemState.PRE_MATCH;
            case ALLIANCE_RED:
                return SystemState.ALLIANCE_RED;
            case ALLIANCE_BLUE:
                return SystemState.ALLIANCE_BLUE;
            case SIGNAL:
                return SystemState.SIGNALING;
            case RESET:
            default:
                return SystemState.RESET;
        }
    }

    private synchronized SystemState handlePreMatch() {
        mRED.set(true);
        mBLUE.set(true);
        mReset.set(true);
        return defaultStateTransfer();
    }

    private synchronized SystemState handleRedAlliance() {
        mRED.set(false);
        return defaultStateTransfer();
    }

    private synchronized SystemState handleBlueAlliance() {
        mBLUE.set(false);
        return defaultStateTransfer();
    }

    private synchronized SystemState handleSignaling() {
        mRED.set(false);
        mBLUE.set(false);
        return defaultStateTransfer();
    }

    private synchronized SystemState handleReset() {
        if (!mHasReset) {
            mRED.set(true);
            mBLUE.set(true);
            mReset.set(false);
            mHasReset = true;
            System.out.println("End of the match! Resetting the LEDs...");
        } else {
            mRED.set(true);
            mBLUE.set(true);
            mReset.set(true);
        }
        mWantedState = WantedState.PRE_MATCH;
        return defaultStateTransfer();
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

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(mLoop);
    }

    public synchronized void setWantedState(WantedState state) {
        mWantedState = state;
    }

    // Internal state of the system
    public enum SystemState {
        PRE_MATCH, ALLIANCE_RED, ALLIANCE_BLUE, SIGNALING, RESET
    }

    public enum WantedState {
        PRE_MATCH, ALLIANCE_RED, ALLIANCE_BLUE, SIGNAL, RESET
    }
}
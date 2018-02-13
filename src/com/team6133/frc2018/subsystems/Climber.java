package com.team6133.frc2018.subsystems;

import com.team6133.frc2018.Constants;
import com.team6133.frc2018.loops.Loop;
import com.team6133.frc2018.loops.Looper;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Climber consists of a telescoping arm, a detachable hook, and a winch.
 * A motor turns a pulley, which raises/lowers the arm.
 * A CIM with a VersaPlanetary Gearbox (50 to 1 ratio) and ratchet acts as a winch in order to climb.
 */
public class Climber extends Subsystem {
    private static Climber mInstance = null;

    public static Climber getInstance() {
        if (mInstance == null) {
            mInstance = new Climber();
        }
        return mInstance;
    }

    public enum WantedState {
        IDLE, EXTEND, RETRACT, CLIMB
    }

    public enum SystemState {
        IDLE, EXTENDING, RETRACTING, CLIMBING
    }

    private WantedState mWantedState;
    private SystemState mSystemState;
    private final Spark mWinchSpark, mPulleySpark;
    private final PowerDistributionPanel mPDP;

    private Climber() {
        mPDP = Intake.getInstance().getPDP();
        mPulleySpark = new Spark(Constants.kClimbPulleyPWM );
        mWinchSpark  = new Spark(Constants.kClimbWinchPWM  );
    }

    private SystemState handleIdle() {
        switch(mWantedState) {
            case CLIMB:
                return SystemState.CLIMBING;
            case EXTEND:
                return SystemState.EXTENDING;
            case RETRACT:
                return SystemState.RETRACTING;
            case IDLE:
            default:
                mWinchSpark.set(0);
                mPulleySpark.set(0);
                return SystemState.IDLE;
        }
    }

    private SystemState handleClimbing() {
        switch (mWantedState) {
            case IDLE:
                return SystemState.IDLE;
            case EXTEND:
                return SystemState.EXTENDING;
            case RETRACT:
                return SystemState.RETRACTING;
            default:
                mWinchSpark.set(1.0);
                return SystemState.CLIMBING;
        }
    }

    private SystemState handleExtending() {
        switch (mWantedState) {
            case CLIMB:
                return SystemState.CLIMBING;
            case IDLE:
                return SystemState.IDLE;
            case RETRACT:
                return SystemState.RETRACTING;
            default:
                mPulleySpark.set(1.0);
                return SystemState.EXTENDING;
        }
    }

    private SystemState handleRetracting() {
        switch (mWantedState) {
            case IDLE:
                return SystemState.IDLE;
            case EXTEND:
                return SystemState.EXTENDING;
            case CLIMB:
                return SystemState.CLIMBING;
            default:
                mPulleySpark.set(-1.0);
                return SystemState.RETRACTING;
        }
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Climber Winch Current", mPDP.getCurrent(3));
    }

    @Override
    public void stop() {
        mWantedState = WantedState.IDLE;
        mSystemState = SystemState.IDLE;
        mWinchSpark.set(0);
        mPulleySpark.set(0);
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        Loop loop = new Loop() {

            @Override
            public void onStart(double timestamp) {
                synchronized (Climber.this) {
                    mSystemState = SystemState.IDLE;
                    mWantedState = WantedState.IDLE;
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Climber.this) {
                    SystemState newState;
                    switch (mSystemState) {
                        case IDLE:
                            newState = handleIdle();
                            break;
                        case CLIMBING:
                            newState = handleClimbing();
                            break;
                        case EXTENDING:
                            newState = handleExtending();
                            break;
                        case RETRACTING:
                            newState = handleRetracting();
                            break;
                        default:
                            newState = handleIdle();
                            break;
                    }
                    if (newState != mSystemState) {
                        System.out.println(timestamp + ": Changed state: " + mSystemState + " -> " + newState);
                        mSystemState = newState;
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        };
        enabledLooper.register(loop);
    }
}

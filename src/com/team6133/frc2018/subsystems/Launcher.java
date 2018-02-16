package com.team6133.frc2018.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.team6133.frc2018.Constants;
import com.team6133.frc2018.loops.Loop;
import com.team6133.frc2018.loops.Looper;
import com.team6133.lib.util.drivers.CANTalonFactory;
import com.team6133.lib.util.drivers.RevRoboticsAirPressureSensor;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Launcher consists of 4 flywheels and a 12" stroke pneumatic cylinder.
 * Once the flywheels have spooled up, then cylinder lifts the cube into the flywheels
 * and the cube launches to the desired height.
 * The Drive subsystem has complimenting methods to assist in cube launching.
 * @see Drive#setAlignLaunch(double)
 */
public class Launcher extends Subsystem {
    private static Launcher mInstance = null;

    public static Launcher getInstance() {
        if (mInstance == null) {
            mInstance = new Launcher();
        }
        return mInstance;
    }

    public final Compressor compressor = new Compressor(0);
    private final DoubleSolenoid mLauncherSolenoid = Constants.makeDoubleSolenoidForId(Constants.kLauncherSolenoidId);

    // @TODO: Plug in the Rev Robotics Air Pressure Sensor to analog input port 0
    private final RevRoboticsAirPressureSensor mAirPressureSensor =
            new RevRoboticsAirPressureSensor(Constants.kRevAirSensorPort);

    public final WPI_TalonSRX mMasterTalon, mSlaveTalon;
    public final Spark mLeftLauncherSpark, mRightLauncherSpark;

    // Internal state of the system
    public enum SystemState {
        IDLE,
        ALIGNING,
        LAUNCHING,
        SPIN_DOWN,
    }

    // Desired function from user
    public enum WantedState {
        IDLE,
        LAUNCH,
        ALIGN
    }

    private SystemState mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.IDLE;

    private final double mLaunchRPM = Constants.kLauncherRPM;
    private final double mSparkRPM  = 1.0;
    private final double kLaunchDelay = 0.1;
    private final double kLaunchTime  = 1.0;
    private final double kPostLaunchDelay = 0.05;
    private final int    kAllowableClosedLoopError = 30;
    private double mThresholdStart = Double.POSITIVE_INFINITY;
    private boolean mWantsLaunch = false;

    private Launcher() {
        compressor.setClosedLoopControl(true);

        mLeftLauncherSpark  = new Spark(Constants.kLauncherLeftPWM  );
        mRightLauncherSpark = new Spark(Constants.kLauncherRightPWM );
        mMasterTalon = CANTalonFactory.createDefaultTalon(Constants.kLauncherMasterId );
        mSlaveTalon  = CANTalonFactory.createDefaultTalon(Constants.kLauncherSlaveId  );

        mMasterTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        mMasterTalon.set(ControlMode.Velocity, 0);
        mMasterTalon.configAllowableClosedloopError(0, kAllowableClosedLoopError,10);
        mMasterTalon.setNeutralMode(NeutralMode.Coast);
        mMasterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 5,10);
        mMasterTalon.config_kP(0, Constants.kLauncherKp, 10);
        mMasterTalon.config_kI(0, Constants.kLauncherKi, 10);
        mMasterTalon.config_kD(0, Constants.kLauncherKd, 10);
        mMasterTalon.config_kF(0, Constants.kLauncherKf, 10);
        mMasterTalon.setSensorPhase(true);

        mSlaveTalon.set(ControlMode.Follower, Constants.kLauncherMasterId);
        mSlaveTalon.setNeutralMode(NeutralMode.Coast);
        mSlaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 5,10);
        mSlaveTalon.setInverted(true);

        mLeftLauncherSpark.set(0);
        mRightLauncherSpark.set(0);

        mLauncherSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public synchronized void setWantedState(WantedState wanted) {
        mWantedState = wanted;
    }

    public synchronized void setWantsLaunch() {mWantsLaunch = true;}

    public synchronized boolean getWantsLaunch() {return mWantsLaunch;}

    private SystemState handleIdle(double timeInState) {
        mLauncherSolenoid.set(DoubleSolenoid.Value.kForward);
        mMasterTalon.set(0);
        mRightLauncherSpark.set(0);
        mLeftLauncherSpark.set(0);

        switch (mWantedState) {
            case ALIGN:
                mThresholdStart = Double.POSITIVE_INFINITY;
                return SystemState.ALIGNING;
            default:
                return SystemState.IDLE;
        }
    }

    private SystemState handleAlign(double timeInState) {
        mLeftLauncherSpark.set(mSparkRPM);
        mRightLauncherSpark.set(-mSparkRPM);
        mMasterTalon.set(ControlMode.Velocity, mLaunchRPM);

        if (mMasterTalon.getClosedLoopError(0) < kAllowableClosedLoopError) {
            if (mThresholdStart == Double.POSITIVE_INFINITY) {
                mThresholdStart = timeInState;
            } else {
                if (timeInState - mThresholdStart > kLaunchDelay && mWantsLaunch) {
                    mThresholdStart = Double.POSITIVE_INFINITY;
                    mWantedState = WantedState.LAUNCH;
                    return SystemState.LAUNCHING;
                }
            }
        } else {
            mThresholdStart = Double.POSITIVE_INFINITY;
        }
        switch (mWantedState) {
            case ALIGN:
                return SystemState.ALIGNING;
            case IDLE:
                return SystemState.IDLE;
            case LAUNCH:
                mThresholdStart = Double.POSITIVE_INFINITY;
                return SystemState.LAUNCHING;
            default:
                return SystemState.ALIGNING;
        }
    }

    private SystemState handleLaunch(double timeInState) {
        mLeftLauncherSpark.set(mSparkRPM);
        mRightLauncherSpark.set(-mSparkRPM);
        mMasterTalon.set(ControlMode.Velocity, mLaunchRPM);
        mLauncherSolenoid.set(DoubleSolenoid.Value.kReverse);

        System.out.println(mMasterTalon.getSelectedSensorVelocity(0));
        if (mThresholdStart == Double.POSITIVE_INFINITY) {
            mThresholdStart = timeInState;
        } else {
            if (timeInState - mThresholdStart > kLaunchTime) {
                mThresholdStart = Double.POSITIVE_INFINITY;
                mWantsLaunch = false;
                return SystemState.SPIN_DOWN;
            }
        }
        return SystemState.LAUNCHING;
    }

    private SystemState handleSpinDown(double timeInState) {
        if (mThresholdStart == Double.POSITIVE_INFINITY) {
            mThresholdStart = timeInState;
        } else {
            if (timeInState - mThresholdStart > kPostLaunchDelay) {
                mThresholdStart = Double.POSITIVE_INFINITY;
                mWantedState = WantedState.IDLE;
                return SystemState.IDLE;
            }
        }
        return SystemState.SPIN_DOWN;
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Air Pressure", mAirPressureSensor.getAirPressurePsi());
        SmartDashboard.putNumber("Launcher Velocity", mMasterTalon.getSelectedSensorVelocity(0));
    }

    @Override
    public void stop() {
        mMasterTalon.set(0);
        mRightLauncherSpark.set(0);
        mLeftLauncherSpark.set(0);
        mSystemState = SystemState.IDLE;
    }

    @Override
    public void zeroSensors() {
        mMasterTalon.setSelectedSensorPosition(0,0,10);
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        Loop loop = new Loop() {
            private double mCurrentStateStartTime;

            @Override
            public void onStart(double timestamp) {
                synchronized (Launcher.this) {
                    mSystemState = SystemState.IDLE;
                    mWantedState = WantedState.IDLE;
                }
                mCurrentStateStartTime = Timer.getFPGATimestamp();
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Launcher.this) {
                    SystemState newState;
                    double timeInState = Timer.getFPGATimestamp() - mCurrentStateStartTime;
                    switch (mSystemState) {
                        case ALIGNING:
                            newState = handleAlign(timeInState);
                            break;
                        case LAUNCHING:
                            newState = handleLaunch(timeInState);
                            break;
                        case SPIN_DOWN:
                            newState = handleSpinDown(timeInState);
                            break;
                        default:
                            newState = handleIdle(timeInState);
                            break;
                    }
                    if (newState != mSystemState) {
                        System.out.println(timestamp + ": Changed state: " + mSystemState + " -> " + newState);
                        mSystemState = newState;
                        mCurrentStateStartTime = Timer.getFPGATimestamp();
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                mWantedState = WantedState.IDLE;
                stop();
            }
        };
        enabledLooper.register(loop);
    }
}

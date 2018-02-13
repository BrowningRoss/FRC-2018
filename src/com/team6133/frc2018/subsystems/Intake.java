package com.team6133.frc2018.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.team6133.lib.util.ConstantsBase;
import com.team6133.lib.util.drivers.CANTalonFactory;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.team6133.lib.util.drivers.CANTalonFactory;
import com.team6133.frc2018.Constants;
import com.team6133.frc2018.loops.Loop;
import com.team6133.frc2018.loops.Looper;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The intake subsystem consists of one PG-71 motor used to raise and lower the arm.  The encoder on the PG-71 is
 * plugged into the corresponding Talon-SRX and used for positioning.  On the arm, there is one BAG motor on each
 * side of the arm.  Each BAG motor is connected to 10:1 VP Gearbox and runs on open loop.
 */

public class Intake extends Subsystem {

    private static final double kIntakeFloorCubeSetpoint = 0.000000;
    private static final double kIntakeStackCubeSetpoint = 0.000000;
    private static final double kExhaustSwitchSetpoint   = 0.000000;
    private static final double kExhaustExchangeSetpoint = 0.000000;
    private static final double kHoldingSetpoint         = 0.000000;
    private static final double kLoadShooterSetpoint     = 0;

    private static final double kIntakeMotorSetpoint     = 1.0;
    private static final double kExhaustMotorSetpoint    = -1.0;
    private static final double kLauncherExhaustSetpoint = -.3;
    private static final double kTransitionDelay = 0.5;
    private static final double kExhaustDelay = 0.1;
    private static final double kStowingDelay = 0.1;
    private static final double kIntakeThreshold = 15;
    private static final double kThresholdTime = 0.25;

    private static final int kAllowableClosedLoopError = 50;
    private static final int kPDPSparkSlot = 10;

    private final DoubleSolenoid mLeftSolenoid, mRightSolenoid;

    private static Intake mInstance = null;

    public static Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
    }

    public enum WantedState {
        ACQUIRE_FLOOR,      // grab cube off the ground
        ACQUIRE_STACK,      // grab cube off the pyramid stack
        HOLDING,            // stowe cube for safe travel
        SCORE_EXCHANGE,     // score cube into the exchange
        SCORE_SWITCH,       // score cube into the switch
        LOAD_SHOOTER,       // transition cube into shooter subsystem
        IDLE                // do nothing, usually as part of a transition to another state OR it's the end of a match.
    }

    public enum SystemState {
        INTAKE_FLOOR,       // arm in lowest position, motors intake
        INTAKE_STACK,       // arm set to mid position, motors intake
        STOWING,            // transition state between INTAKE_* and STOWED where the motor
                            // intakes so the cube doesn't sly out as the subsystem pivots up.
        STOWED,             // arm up to high position, motors stopped
        EXHAUST_SWITCH,     // arm in mid position, motors in reverse
        EXHAUST_EXCHANGE,    // arm in lowest position, motors in reverse
        EXHAUST_SHOOTER     // arm in highest position, motors in reverse
    }

    private final WPI_TalonSRX mArmTalon;
    private final Spark mLeftSpark, mRightSpark;

    private WantedState mWantedState;
    private SystemState mSystemState;
    private double mThresholdStart = Double.POSITIVE_INFINITY;
    private boolean mHasCube = false;
    private boolean mWantsExhaust = false;
    private PowerDistributionPanel mPDP;

    private Intake() {
        mArmTalon   = CANTalonFactory.createDefaultTalon(Constants.kIntakeArmId         );
        mRightSpark = new Spark(Constants.kIntakeRightPWM   );
        mLeftSpark  = new Spark(Constants.kIntakeLeftPWM    );

        mLeftSolenoid  = Constants.makeDoubleSolenoidForId(Constants.kArmLeftSolenoidId  );
        mRightSolenoid = Constants.makeDoubleSolenoidForId(Constants.kArmRightSolenoidId );

        mArmTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 5, 10);
        mArmTalon.setNeutralMode(NeutralMode.Brake);
        mArmTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0,10);
        mArmTalon.configAllowableClosedloopError(0,kAllowableClosedLoopError,10);
        mArmTalon.config_kP(0,Constants.kIntakeKp, 10);
        mArmTalon.config_kI(0,Constants.kIntakeKi, 10);
        mArmTalon.config_kD(0,Constants.kIntakeKd, 10);
        mArmTalon.config_kF(0,Constants.kIntakeKf, 10);
        mArmTalon.setSelectedSensorPosition(0,0,10);

        mArmTalon.set(ControlMode.Position, 0);

        mRightSpark.setSafetyEnabled(true);
        mLeftSpark.setSafetyEnabled(true);
        mLeftSpark.set(0);
        mRightSpark.set(0);

        mLeftSolenoid.set( DoubleSolenoid.Value.kOff);
        mRightSolenoid.set(DoubleSolenoid.Value.kOff);

        mPDP = new PowerDistributionPanel();
    }

    public PowerDistributionPanel getPDP() {return mPDP;}

    public boolean hasCube() {return mHasCube;}

    public void overrideHasCube() {mHasCube = !mHasCube;}

    public synchronized void setWantedState(WantedState wanted) {
        mWantedState = wanted;
    }

    public synchronized void reset() {
        mWantedState = WantedState.HOLDING;
        mSystemState = SystemState.STOWED;
    }

    private SystemState handleFloorIntake(double timeInState) {
        switch (mWantedState) {
            case HOLDING:
                mThresholdStart = Double.POSITIVE_INFINITY;
                return SystemState.STOWING;
            default:
                mArmTalon.set(ControlMode.Position, kIntakeFloorCubeSetpoint);
                mLeftSolenoid.set( DoubleSolenoid.Value.kForward);
                mRightSolenoid.set(DoubleSolenoid.Value.kForward);
                mLeftSpark.set(kIntakeMotorSetpoint);
                mRightSpark.set(-kIntakeMotorSetpoint);
                if ( (int) (Math.floor(10*timeInState))% 2 == 0)
                    System.out.println("Spark Current Intake Floor (amps): " + mPDP.getCurrent(kPDPSparkSlot));

                if (mPDP.getCurrent(kPDPSparkSlot) > kIntakeThreshold) {
                    if (timeInState - mThresholdStart > kThresholdTime) {
                        // @TODO: LED indicator
                        mHasCube = true;
                        return SystemState.STOWING;
                    } else {
                        if (mThresholdStart == Double.POSITIVE_INFINITY) {
                            mThresholdStart = timeInState;
                        }
                    }
                } else {
                    mThresholdStart = Double.POSITIVE_INFINITY;
                    // @TODO: Led indicator
                }
                return SystemState.INTAKE_FLOOR;
        }
    }

    private SystemState handleStackIntake(double timeInState) {
        switch (mWantedState) {
            case HOLDING:
                mThresholdStart = Double.POSITIVE_INFINITY;
                return SystemState.STOWING;
            default:
                mArmTalon.set(ControlMode.Position, kIntakeStackCubeSetpoint);
                mLeftSolenoid.set( DoubleSolenoid.Value.kForward);
                mRightSolenoid.set(DoubleSolenoid.Value.kForward);
                mLeftSpark.set(kIntakeMotorSetpoint);
                mRightSpark.set(-kIntakeMotorSetpoint);
                if ( (int) (Math.floor(10*timeInState))% 2 == 0)
                    System.out.println("Spark Current Intake Stack (amps): " + mPDP.getCurrent(kPDPSparkSlot));

                if (mPDP.getCurrent(kPDPSparkSlot) > kIntakeThreshold) {
                    if (timeInState - mThresholdStart > kThresholdTime) {
                        // @TODO: LED indicator
                        mHasCube = true;
                        return SystemState.STOWING;
                    } else {
                        if (mThresholdStart == Double.POSITIVE_INFINITY) {
                            mThresholdStart = timeInState;
                        }
                    }
                } else {
                    mThresholdStart = Double.POSITIVE_INFINITY;
                    // @TODO: Led indicator
                }
                return SystemState.INTAKE_STACK;
        }
    }

    private SystemState handleExhaustExchange(double timeInState) {
        mArmTalon.set(ControlMode.Position, kExhaustExchangeSetpoint);
        mLeftSolenoid.set( DoubleSolenoid.Value.kForward);
        mRightSolenoid.set(DoubleSolenoid.Value.kForward);
        if (mArmTalon.getClosedLoopError(0) < kAllowableClosedLoopError) {
            if (mThresholdStart == Double.POSITIVE_INFINITY) {
                mThresholdStart = timeInState;
            } else {
                if (timeInState - mThresholdStart > kExhaustDelay && mWantsExhaust) {
                    mLeftSpark.set(kExhaustMotorSetpoint);
                    mRightSpark.set(-kExhaustMotorSetpoint);
                    mHasCube = false;
                    mWantsExhaust = false;
                } else {
                    return SystemState.STOWING;
                }
            }
        } else {
            mThresholdStart = Double.POSITIVE_INFINITY;
        }

        if (timeInState < kTransitionDelay) {
            return SystemState.EXHAUST_EXCHANGE;
        }

        switch (mWantedState) {
            case SCORE_EXCHANGE:
                return SystemState.EXHAUST_EXCHANGE;
            case SCORE_SWITCH:
                mThresholdStart = Double.POSITIVE_INFINITY;
                mArmTalon.set(ControlMode.Position, kExhaustSwitchSetpoint);
                return SystemState.EXHAUST_SWITCH;
            case ACQUIRE_FLOOR:
                mThresholdStart = Double.POSITIVE_INFINITY;
                mArmTalon.set(ControlMode.Position, kIntakeFloorCubeSetpoint);
                return SystemState.INTAKE_FLOOR;
            default:
                mThresholdStart = Double.POSITIVE_INFINITY;
                mArmTalon.set(ControlMode.Position, kHoldingSetpoint);
                return SystemState.STOWING;
        }
    }

    public synchronized void setWantsExhaust() {
        mWantsExhaust = true;
    }

    private SystemState handleExhaustSwitch(double timeInState) {
        mArmTalon.set(ControlMode.Position, kExhaustSwitchSetpoint);
        mLeftSolenoid.set( DoubleSolenoid.Value.kForward);
        mRightSolenoid.set(DoubleSolenoid.Value.kForward);

        if (mArmTalon.getClosedLoopError(0) < kAllowableClosedLoopError) {
            if (mThresholdStart == Double.POSITIVE_INFINITY) {
                mThresholdStart = timeInState;
            } else {
                if (timeInState - mThresholdStart > kExhaustDelay && mWantsExhaust) {
                    mLeftSpark.set(kExhaustMotorSetpoint);
                    mRightSpark.set(-kExhaustMotorSetpoint);
                    mHasCube = false;
                    mWantsExhaust = false;
                } else {
                    mRightSpark.set(0);
                    mLeftSpark.set(0);
                    return SystemState.STOWING;
                }
            }
        } else {
            mThresholdStart = Double.POSITIVE_INFINITY;
        }

        if (timeInState < kTransitionDelay) {
            return SystemState.EXHAUST_SWITCH;
        }

        switch (mWantedState) {
            case SCORE_EXCHANGE:
                mThresholdStart = Double.POSITIVE_INFINITY;
                mArmTalon.set(ControlMode.Position, kExhaustExchangeSetpoint);
                return SystemState.EXHAUST_EXCHANGE;
            case SCORE_SWITCH:
                return SystemState.EXHAUST_SWITCH;
            case ACQUIRE_FLOOR:
                mThresholdStart = Double.POSITIVE_INFINITY;
                mArmTalon.set(ControlMode.Position, kIntakeFloorCubeSetpoint);
                return SystemState.INTAKE_FLOOR;
            default:
                mThresholdStart = Double.POSITIVE_INFINITY;
                mArmTalon.set(ControlMode.Position, kHoldingSetpoint);
                return SystemState.STOWING;
        }
    }

    private SystemState handleExhaustShooter(double timeInState) {
        mArmTalon.set(ControlMode.Position, kLoadShooterSetpoint);
        mLeftSolenoid.set( DoubleSolenoid.Value.kReverse);
        mRightSolenoid.set(DoubleSolenoid.Value.kReverse);
        if (mArmTalon.getClosedLoopError(0) < kAllowableClosedLoopError) {
            if (mThresholdStart == Double.POSITIVE_INFINITY) {
                mThresholdStart = timeInState;
            } else {
                if (timeInState - mThresholdStart > kExhaustDelay) {
                    mLeftSpark.set(kLauncherExhaustSetpoint);
                    mRightSpark.set(-kLauncherExhaustSetpoint);
                    mHasCube = false;
                } else {
                    mLeftSpark.set(0);
                    mRightSpark.set(0);
                }
            }
        } else {
            mThresholdStart = Double.POSITIVE_INFINITY;
        }
        switch(mWantedState) {
            case LOAD_SHOOTER:
                return SystemState.EXHAUST_SHOOTER;
            case IDLE:
            // fallthrough intended
            case HOLDING:
                mThresholdStart = Double.POSITIVE_INFINITY;
                return SystemState.STOWING;
            case SCORE_EXCHANGE:
                mThresholdStart = Double.POSITIVE_INFINITY;
                return SystemState.EXHAUST_EXCHANGE;
            case SCORE_SWITCH:
                mThresholdStart = Double.POSITIVE_INFINITY;
                return SystemState.EXHAUST_SWITCH;
            case ACQUIRE_FLOOR:
                mThresholdStart = Double.POSITIVE_INFINITY;
                return SystemState.INTAKE_FLOOR;
            case ACQUIRE_STACK:
                mThresholdStart = Double.POSITIVE_INFINITY;
                return SystemState.INTAKE_STACK;
            default:
                mThresholdStart = Double.POSITIVE_INFINITY;
                return SystemState.STOWED;
        }
    }

    private SystemState handleStowing(double timeInState) {
        mArmTalon.set(ControlMode.Position, kHoldingSetpoint);
        mLeftSolenoid.set( DoubleSolenoid.Value.kReverse);
        mRightSolenoid.set(DoubleSolenoid.Value.kReverse);
        if (mArmTalon.getClosedLoopError(0) < kAllowableClosedLoopError) {
            if (mThresholdStart == Double.POSITIVE_INFINITY) {
                mThresholdStart = timeInState;
            } else {
                if (timeInState - mThresholdStart > kStowingDelay) {
                    mLeftSpark.set(kIntakeMotorSetpoint);
                    mRightSpark.set(-kIntakeMotorSetpoint);
                } else {
                    return SystemState.STOWED;
                }
            }
        } else {
            mThresholdStart = Double.POSITIVE_INFINITY;
        }
        return SystemState.STOWING;
    }

    private SystemState handleStowed(double timeInState) {
        mLeftSolenoid.set( DoubleSolenoid.Value.kOff);
        mRightSolenoid.set(DoubleSolenoid.Value.kOff);
        switch(mWantedState) {
            case ACQUIRE_FLOOR:
                mThresholdStart = Double.POSITIVE_INFINITY;
                return SystemState.INTAKE_FLOOR;
            case ACQUIRE_STACK:
                mThresholdStart = Double.POSITIVE_INFINITY;
                return SystemState.INTAKE_STACK;
            case SCORE_EXCHANGE:
                mThresholdStart = Double.POSITIVE_INFINITY;
                return SystemState.EXHAUST_EXCHANGE;
            case SCORE_SWITCH:
                mThresholdStart = Double.POSITIVE_INFINITY;
                return SystemState.EXHAUST_SWITCH;
            case LOAD_SHOOTER:
                mThresholdStart = Double.POSITIVE_INFINITY;
                return SystemState.EXHAUST_SHOOTER;
            default:
                mArmTalon.set(ControlMode.Position, kHoldingSetpoint);
                mRightSpark.set(0);
                mLeftSpark.set(0);
                return SystemState.STOWED;
        }
    }

    @Override
    public void writeToLog() {

    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Intake Arm Position", mArmTalon.getSelectedSensorPosition(0));

        SmartDashboard.putNumber("Intake Left Current", mPDP.getCurrent(kPDPSparkSlot));
        SmartDashboard.putNumber("Intake Right Current", mPDP.getCurrent(4));
    }

    @Override
    public void stop() {
        setWantedState(WantedState.IDLE);
    }

    @Override
    public void zeroSensors() {
        mArmTalon.setSelectedSensorPosition(0,0,10);
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        Loop loop = new Loop() {
            private double mCurrentStateStartTime;

            @Override
            public void onStart(double timestamp) {
                synchronized (Intake.this) {
                    mSystemState = SystemState.STOWING;
                    mWantedState = WantedState.HOLDING;
                }
            mCurrentStateStartTime = Timer.getFPGATimestamp();
        }

        @Override
            public void onLoop(double timestamp) {
                synchronized (Intake.this) {
                    SystemState newState = mSystemState;
                    double timeInState = Timer.getFPGATimestamp() - mCurrentStateStartTime;
                    switch (mSystemState) {
                        case INTAKE_FLOOR:
                            newState = handleFloorIntake(timeInState);
                            break;
                        case INTAKE_STACK:
                            newState = handleStackIntake(timeInState);
                            break;
                        case EXHAUST_EXCHANGE:
                            newState = handleExhaustExchange(timeInState);
                            break;
                        case EXHAUST_SWITCH:
                            newState = handleExhaustSwitch(timeInState);
                            break;
                        case EXHAUST_SHOOTER:
                            newState = handleExhaustShooter(timeInState);
                            break;
                        case STOWING:
                            newState = handleStowing(timeInState);
                            break;
                        case STOWED:
                            newState = handleStowed(timeInState);
                            break;
                        default:
                            System.out.println("Unexpected Intake system state: " + mSystemState);
                            newState = mSystemState;
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
                mSystemState = SystemState.STOWING;
                stop();
            }
        };
        enabledLooper.register(loop);
    }
}

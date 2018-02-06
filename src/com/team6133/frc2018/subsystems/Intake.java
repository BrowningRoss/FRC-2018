package com.team6133.frc2018.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.team6133.lib.util.ConstantsBase;
import com.team6133.lib.util.drivers.CANTalonFactory;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.team6133.lib.util.drivers.CANTalonFactory;
import com.team6133.frc2018.Constants;
import com.team6133.frc2018.loops.Loop;
import com.team6133.frc2018.loops.Looper;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The intake subsystem consists of one PG-71 motor used to raise and lower the arm.  The encoder on the PG-71 is
 * plugged into the corresponding Talon-SRX and used for positioning.  On the arm, there is one BAG motor on each
 * side of the arm.  Each BAG motor is connected to 10:1 VP Gearbox and runs on open loop.
 */

public class Intake extends Subsystem {

    public static double kIntakeFloorCubeSetpoint = 0.000000;
    public static double kIntakeStackCubeSetpoint = 0.000000;
    public static double kExhaustSwitchSetpoint = 0.00000;
    public static double kExhaustExchangeSetpoint = 0.00000;
    public static double kHoldingSetpoint = 0.000000;
    public static double kLoadShooterSetpoint = 0.00000;

    public static double kIntakeMotorSetpoint = 1.0;
    public static double kExhaustMotorSetpoint = -1.0;
    public static double kTransitionDelay = 0.5;
    public static double kExhaustDelay = 0.1;
    public static double kStowingDelay = 0.1;
    public static double kIntakeThreshold = 15;
    public static double kThresholdTime = 0.25;

    public static int kAllowableClosedLoopError = 50;

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

    private final WPI_TalonSRX mArmTalon;//+++, mLeftTalon, mRightTalon;

    private WantedState mWantedState;
    private SystemState mSystemState;
    private double mThresholdStart;
    private boolean mHasCube = false;

    private Intake() {
        mArmTalon   = CANTalonFactory.createDefaultTalon(Constants.kIntakeArmId         );
        //+++mLeftTalon  = CANTalonFactory.createDefaultTalon(Constants.kIntakeLeftWheelId   );
        //+++mRightTalon = CANTalonFactory.createDefaultTalon(Constants.kIntakeRightWheelId  );

        mArmTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 5, 10);
        mArmTalon.setNeutralMode(NeutralMode.Coast);
        mArmTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0,10);
        mArmTalon.configAllowableClosedloopError(0,kAllowableClosedLoopError,10);
        mArmTalon.setSelectedSensorPosition(0,0,10);

        mArmTalon.set(ControlMode.Position, 0);
        /*+++
        mLeftTalon.setStatusFramePeriod(StatusFrame.Status_1_General, 15, 100);
        mLeftTalon.set(ControlMode.PercentOutput, 0);

        mRightTalon.setStatusFramePeriod(StatusFrame.Status_1_General, 15, 100);
        mRightTalon.set(ControlMode.PercentOutput, 0);
        +++*/
    }

    public boolean hasCube() {return mHasCube;}

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
                //+++mLeftTalon.set(kIntakeMotorSetpoint);
                //+++mRightTalon.set(-kIntakeMotorSetpoint);

                if (true){//+++mLeftTalon.getOutputCurrent() > kIntakeThreshold) {
                    if (timeInState - mThresholdStart > kThresholdTime) {
                        // @TODO: LED indicator
                        mHasCube = true;
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
                //+++mLeftTalon.set(kIntakeMotorSetpoint);
                //+++mRightTalon.set(-kIntakeMotorSetpoint);

                if (true){//+++mLeftTalon.getOutputCurrent() > kIntakeThreshold) {
                    if (timeInState - mThresholdStart > kThresholdTime) {
                        // @TODO: LED indicator
                        mHasCube = true;
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

        if (mArmTalon.getClosedLoopError(0) < kAllowableClosedLoopError) {
            if (mThresholdStart == Double.POSITIVE_INFINITY) {
                mThresholdStart = timeInState;
            } else {
                if (timeInState - mThresholdStart > kExhaustDelay) {
                    //+++motors on
                    mHasCube = false;
                } else {
                    //+++motors off
                }
            }
        } else {
            mThresholdStart = Double.POSITIVE_INFINITY;
        }

        if (timeInState < kTransitionDelay) {
            return SystemState.EXHAUST_EXCHANGE;
        }

        if (timeInState > kExhaustDelay) {

        } else {

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

    private SystemState handleExhaustSwitch(double timeInState) {
        mArmTalon.set(ControlMode.Position, kExhaustSwitchSetpoint);

        if (mArmTalon.getClosedLoopError(0) < kAllowableClosedLoopError) {
            if (mThresholdStart == Double.POSITIVE_INFINITY) {
                mThresholdStart = timeInState;
            } else {
                if (timeInState - mThresholdStart > kExhaustDelay) {
                    //+++motors on
                    mHasCube = false;
                } else {
                    //+++motors off
                }
            }
        } else {
            mThresholdStart = Double.POSITIVE_INFINITY;
        }

        if (timeInState < kTransitionDelay) {
            return SystemState.EXHAUST_EXCHANGE;
        }

        if (timeInState > kExhaustDelay) {

        } else {

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

        if (mArmTalon.getClosedLoopError(0) < kAllowableClosedLoopError) {
            if (mThresholdStart == Double.POSITIVE_INFINITY) {
                mThresholdStart = timeInState;
            } else {
                if (timeInState - mThresholdStart > kExhaustDelay) {
                    //+++motors on
                    mHasCube = false;
                } else {
                    //+++motors off
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

        if (mArmTalon.getClosedLoopError(0) < kAllowableClosedLoopError) {
            if (mThresholdStart == Double.POSITIVE_INFINITY) {
                mThresholdStart = timeInState;
            } else {
                if (timeInState - mThresholdStart > kStowingDelay) {
                    //+++motors on
                } else {
                    //+++motors off
                    return SystemState.STOWED;
                }
            }
        } else {
            mThresholdStart = Double.POSITIVE_INFINITY;
        }
        return SystemState.STOWING;
    }

    private SystemState handleStowed(double timeInState) {
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

                if (mArmTalon.getClosedLoopError(0) < kAllowableClosedLoopError) {
                    if (mThresholdStart == Double.POSITIVE_INFINITY) {
                        mThresholdStart = timeInState;
                    } else {
                        if (timeInState - mThresholdStart > 0.5) {
                            //+++motors off
                        } else {
                            //+++motors on
                        }
                    }
                } else {
                    mThresholdStart = Double.POSITIVE_INFINITY;
                }
                return SystemState.STOWED;
        }
    }

    @Override
    public void writeToLog() {

    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Arm Intake Current", mArmTalon.getOutputCurrent());
        //+++SmartDashboard.putNumber("Intake Left Current", mLeftTalon.getOutputCurrent());
        //+++SmartDashboard.putNumber("Intake Right Current", mRightTalon.getOutputCurrent());
    }

    @Override
    public void stop() {
        setWantedState(WantedState.IDLE);
    }

    @Override
    public void zeroSensors() {

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

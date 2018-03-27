package com.team6133.frc2018.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.team6133.lib.util.drivers.CANTalonFactory;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.team6133.frc2018.Constants;
import com.team6133.frc2018.loops.Loop;
import com.team6133.frc2018.loops.Looper;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static com.team6133.frc2018.subsystems.Intake.PistonState.CLOSE;
import static com.team6133.frc2018.subsystems.Intake.PistonState.OPEN;

/**
 * The intake subsystem consists of one PG-71 motor used to raise and lower the arm.  The encoder on the PG-71 is
 * plugged into the corresponding Talon-SRX and used for positioning.  On the arm, there is one BAG motor on each
 * side of the arm.  Each BAG motor is connected to 10:1 VP Gearbox and runs on open loop.
 */

public class Intake extends Subsystem {

    private static final double kIntakeFloorCubeSetpoint = 675;
    private static final double kIntakeStackCubeSetpoint = 350;
    private static final double kExhaustSwitchSetpoint   = 350;
    private static final double kExhaustExchangeSetpoint = 500;
    private static final double kHoldingSetpoint         = 0.000000;
    private static final double kLoadShooterSetpoint     = 0;

    private static final double kIntakeMotorSetpoint     = .95;
    private static final double kExhaustMotorSetpoint    = .85;
    private static final double kStowingMotorSetpoint    = -.5;
    private static final double kLauncherExhaustSetpoint = .5;
    private static final double kTransitionDelay = 1.0;
    private static final double kExhaustDelay = 1;
    private static final double kLoadShooterDelay = .1;
    private static final double kStowingDelay = 0.1;
    private static final double kIntakeThreshold = 17;
    private static final double kThresholdTime = .6133;

    private static final int kAllowableClosedLoopError = 5;
    private static final int kPDPSparkSlot = 5;

    private final DoubleSolenoid mLeftSolenoid;

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
        INTAKE_DONE,
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

    public enum PistonState {
        OPEN, CLOSE
    }

    private final WPI_TalonSRX mArmTalon;
    private final Spark mLeftSpark, mRightSpark;
    private final DigitalInput mLimitSwitch;
    private final LED mLED = LED.getInstance();

    private WantedState mWantedState;
    private SystemState mSystemState;
    private PistonState mPistonState = CLOSE;
    private double mThresholdStart = Double.POSITIVE_INFINITY;
    private double mThresholdEnd = Double.POSITIVE_INFINITY;
    private boolean mHasCube = false;
    private boolean mWantsExhaust = false;
    private PowerDistributionPanel mPDP;

    private Intake() {
        mArmTalon   = CANTalonFactory.createDefaultTalon(Constants.kIntakeArmId         );
        mRightSpark = new Spark(Constants.kIntakeRightPWM   );
        mLeftSpark  = new Spark(Constants.kIntakeLeftPWM    );

        mLeftSolenoid  = Constants.makeDoubleSolenoidForId(Constants.kArmLeftSolenoidId  );

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
        mArmTalon.setInverted(true);
        mArmTalon.setSensorPhase(true);

        mLeftSpark.set(0);
        mRightSpark.set(0);

        mLimitSwitch = new DigitalInput(0);

        mLeftSolenoid.set( DoubleSolenoid.Value.kForward);

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
        mPistonState = CLOSE;
    }

    public void setPistonState(PistonState state) {
        if (mPistonState != state) {
            mPistonState = state;
            switch (state) {
                case OPEN:
                    mLeftSolenoid.set(DoubleSolenoid.Value.kReverse);
                    return;
                case CLOSE:
                    mLeftSolenoid.set(DoubleSolenoid.Value.kForward);
            }
        }
    }

    public PistonState getNewPistonState() {
        if (mPistonState == CLOSE)
            return OPEN;
        return CLOSE;
    }

    private SystemState handleFloorIntake(double timeInState) {
        switch (mWantedState) {
            case HOLDING:
                mThresholdStart = Double.POSITIVE_INFINITY;
                return SystemState.STOWING;
            case SCORE_EXCHANGE:
                mThresholdStart = Double.POSITIVE_INFINITY;
                return SystemState.EXHAUST_EXCHANGE;
            case INTAKE_DONE:
                return SystemState.STOWING;
            default:
                mArmTalon.set(ControlMode.Position, kIntakeFloorCubeSetpoint);

                mLeftSpark.set(kIntakeMotorSetpoint);
                mRightSpark.set(-kIntakeMotorSetpoint);

                if (mPDP.getCurrent(kPDPSparkSlot) > kIntakeThreshold) {
                    if (mThresholdStart == Double.POSITIVE_INFINITY) {
                        mThresholdStart = timeInState;
                    } else if (timeInState - mThresholdStart > kThresholdTime) {
                        mLED.setWantedState(LED.WantedState.SIGNAL);
                    }
                }
                /*
                if (mPDP.getCurrent(kPDPSparkSlot) > kIntakeThreshold && !mHasCube) {
                    mHasCube = true;
                    mThresholdStart = timeInState;
                } else if (!mHasCube) {
                    mThresholdStart = Double.POSITIVE_INFINITY;
                }
                if (mHasCube) {
                    if (timeInState - mThresholdStart > kThresholdTime) {
                        //mArmTalon.config_kP(0, Constants.kIntakeKp, 0);
                        mThresholdStart = Double.POSITIVE_INFINITY;
                        //if (mPDP.getCurrent(kPDPSparkSlot) > kIntakeThreshold)
                        //   return SystemState.STOWING;
                        //else
                        //   mHasCube = false;
                    } else {
                        if (mThresholdStart == Double.POSITIVE_INFINITY) {
                            mThresholdStart = timeInState;
                        }
                    }
                }*/
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
                //mLeftSolenoid.set( DoubleSolenoid.Value.kReverse);
                //mRightSolenoid.set(DoubleSolenoid.Value.kReverse);
                mLeftSpark.set(kIntakeMotorSetpoint);
                mRightSpark.set(-kIntakeMotorSetpoint);

                if (mPDP.getCurrent(kPDPSparkSlot) > kIntakeThreshold) {
                    mHasCube = true;
                } else {
                    mThresholdStart = Double.POSITIVE_INFINITY;
                    mHasCube = false;
                    // @TODO: Led indicator
                }
                if (mHasCube) {
                    if (timeInState - mThresholdStart > kThresholdTime) {
                        // @TODO: LED indicator
                        mThresholdStart = Double.POSITIVE_INFINITY;
                        return SystemState.STOWING;
                    } else {
                        if (mThresholdStart == Double.POSITIVE_INFINITY) {
                            mThresholdStart = timeInState;
                        }
                    }
                }
                return SystemState.INTAKE_STACK;
        }
    }

    private SystemState handleExhaustExchange(double timeInState) {
        mArmTalon.set(ControlMode.Position, kExhaustExchangeSetpoint);
        //mLeftSolenoid.set( DoubleSolenoid.Value.kReverse);
        //mRightSolenoid.set(DoubleSolenoid.Value.kReverse);
        if (mArmTalon.getClosedLoopError(0) < kAllowableClosedLoopError) {
            if (mThresholdStart == Double.POSITIVE_INFINITY) {
                mThresholdStart = timeInState;
                System.out.println("Entered Set Point");
            } else {
                if (timeInState - mThresholdStart > kExhaustDelay && mWantsExhaust) {
                    mLeftSpark.set(kExhaustMotorSetpoint);
                    mRightSpark.set(-kExhaustMotorSetpoint);
                    if (mThresholdEnd == Double.POSITIVE_INFINITY) {
                        mThresholdEnd = timeInState;
                    }
                } else if (!mWantsExhaust){
                    mRightSpark.set(-kStowingMotorSetpoint);
                    mLeftSpark.set(kStowingMotorSetpoint);
                }
            }
        } else {
            mThresholdStart = Double.POSITIVE_INFINITY;
            mRightSpark.set(-kStowingMotorSetpoint);
            mLeftSpark.set(kStowingMotorSetpoint);
        }

        if (timeInState - mThresholdEnd < kTransitionDelay && mWantsExhaust) {
            mLeftSpark.set(kExhaustMotorSetpoint);
            mRightSpark.set(-kExhaustMotorSetpoint);
            return SystemState.EXHAUST_EXCHANGE;
        } else if (mWantedState == WantedState.SCORE_EXCHANGE && mWantsExhaust){
            mWantsExhaust = false;
            mHasCube = false;
            mWantedState = WantedState.HOLDING;
            mThresholdStart = Double.POSITIVE_INFINITY;
            mThresholdEnd = Double.POSITIVE_INFINITY;
            return SystemState.STOWING;
        }

        switch (mWantedState) {
            case SCORE_SWITCH:
                mThresholdStart = Double.POSITIVE_INFINITY;
                mThresholdEnd = Double.POSITIVE_INFINITY;
                mArmTalon.set(ControlMode.Position, kExhaustSwitchSetpoint);
                return SystemState.EXHAUST_SWITCH;
            case ACQUIRE_FLOOR:
                mThresholdStart = Double.POSITIVE_INFINITY;
                mThresholdEnd = Double.POSITIVE_INFINITY;
                mArmTalon.set(ControlMode.Position, kIntakeFloorCubeSetpoint);
                return SystemState.INTAKE_FLOOR;
            case SCORE_EXCHANGE:
                return SystemState.EXHAUST_EXCHANGE;
            default:
                mThresholdStart = Double.POSITIVE_INFINITY;
                mThresholdEnd = Double.POSITIVE_INFINITY;
                mArmTalon.set(ControlMode.Position, kHoldingSetpoint);
                return SystemState.STOWING;
        }
    }

    public synchronized void setWantsExhaust() {
        mWantsExhaust = true;
    }

    public synchronized WantedState getWantedState() {return mWantedState;}

    private SystemState handleExhaustSwitch(double timeInState) {
        mArmTalon.config_kP(0,50, 0);
        mArmTalon.config_kD(0,5,0);
        //mLeftSolenoid.set( DoubleSolenoid.Value.kReverse);
        //mRightSolenoid.set(DoubleSolenoid.Value.kReverse);
        mArmTalon.set(ControlMode.Position, kExhaustSwitchSetpoint);

        if (mArmTalon.getClosedLoopError(0) < kAllowableClosedLoopError) {
            if (mThresholdStart == Double.POSITIVE_INFINITY) {
                mThresholdStart = timeInState;
            } else {
                if (timeInState - mThresholdStart > kExhaustDelay && mWantsExhaust) {
                    mLeftSpark.set(kExhaustMotorSetpoint);
                    mRightSpark.set(-kExhaustMotorSetpoint);
                    if (mThresholdEnd == Double.POSITIVE_INFINITY) {
                        mThresholdEnd = timeInState;
                    }
                } else if (!mWantsExhaust) {
                    mRightSpark.set(-kStowingMotorSetpoint);
                    mLeftSpark.set(kStowingMotorSetpoint);
                }
            }
        } else {
            mThresholdStart = Double.POSITIVE_INFINITY;
            mRightSpark.set(-kStowingMotorSetpoint);
            mLeftSpark.set(kStowingMotorSetpoint);
        }

        if (timeInState < kTransitionDelay + mThresholdEnd && mWantsExhaust) {
            mLeftSpark.set(kExhaustMotorSetpoint);
            mRightSpark.set(-kExhaustMotorSetpoint);
            return SystemState.EXHAUST_SWITCH;
        } else if (mWantedState == WantedState.SCORE_SWITCH && mWantsExhaust){
            mWantsExhaust = false;
            mHasCube = false;
            mWantedState = WantedState.HOLDING;
            mThresholdStart = Double.POSITIVE_INFINITY;
            mArmTalon.config_kP(0, Constants.kIntakeKp, 0);
            mArmTalon.config_kD(0,Constants.kIntakeKd,0);
            return SystemState.STOWING;
        }

        switch (mWantedState) {
            case SCORE_EXCHANGE:
                mThresholdStart = Double.POSITIVE_INFINITY;
                mThresholdEnd = Double.POSITIVE_INFINITY;
                mArmTalon.set(ControlMode.Position, kExhaustExchangeSetpoint);
                return SystemState.EXHAUST_EXCHANGE;
            case ACQUIRE_FLOOR:
                mThresholdStart = Double.POSITIVE_INFINITY;
                mThresholdEnd = Double.POSITIVE_INFINITY;
                mArmTalon.set(ControlMode.Position, kIntakeFloorCubeSetpoint);
                return SystemState.INTAKE_FLOOR;
            case SCORE_SWITCH:
                return SystemState.EXHAUST_SWITCH;
            default:
                mThresholdStart = Double.POSITIVE_INFINITY;
                mThresholdEnd = Double.POSITIVE_INFINITY;
                mArmTalon.set(ControlMode.Position, kHoldingSetpoint);
                return SystemState.STOWING;
        }
    }

    private SystemState handleExhaustShooter(double timeInState) {
        mArmTalon.set(ControlMode.Position, kLoadShooterSetpoint);
        //mLeftSolenoid.set( DoubleSolenoid.Value.kForward);
        //mRightSolenoid.set(DoubleSolenoid.Value.kForward);
        if (mArmTalon.getClosedLoopError(0) < kAllowableClosedLoopError) {
            if (mThresholdStart == Double.POSITIVE_INFINITY) {
                mThresholdStart = timeInState;
            } else {
                if (timeInState - mThresholdStart > kLoadShooterDelay) {
                    mLeftSpark.set(kLauncherExhaustSetpoint);
                    mRightSpark.set(-kLauncherExhaustSetpoint);
                    mWantsExhaust = true;
                    if (mThresholdEnd == Double.POSITIVE_INFINITY)
                        mThresholdEnd = timeInState;
                } else if (!mWantsExhaust){
                    mRightSpark.set(-kStowingMotorSetpoint);
                    mLeftSpark.set(kStowingMotorSetpoint);
                }
            }
        } else {
            mThresholdStart = Double.POSITIVE_INFINITY;
            mRightSpark.set(-kStowingMotorSetpoint);
            mLeftSpark.set(kStowingMotorSetpoint);
        }
        if (timeInState < kTransitionDelay + mThresholdEnd && mWantsExhaust) {
            mLeftSpark.set(kLauncherExhaustSetpoint);
            mRightSpark.set(-kLauncherExhaustSetpoint);
            return SystemState.EXHAUST_SHOOTER;
        } else if (mWantedState == WantedState.LOAD_SHOOTER && mWantsExhaust){
            mHasCube = false;
            mWantsExhaust = false;
            mThresholdStart = Double.POSITIVE_INFINITY;
            mWantedState = WantedState.HOLDING;
            return SystemState.STOWING;
        } else if (mWantedState == WantedState.LOAD_SHOOTER){
            mLeftSpark.set(kStowingMotorSetpoint);
            mRightSpark.set(-kStowingMotorSetpoint);
            return SystemState.EXHAUST_SHOOTER;
        }
        switch(mWantedState) {
            case IDLE:
            // fallthrough intended
            case HOLDING:
                mThresholdStart = Double.POSITIVE_INFINITY;
                mThresholdEnd = Double.POSITIVE_INFINITY;
                return SystemState.STOWING;
            case SCORE_EXCHANGE:
                mThresholdStart = Double.POSITIVE_INFINITY;
                mThresholdEnd = Double.POSITIVE_INFINITY;
                return SystemState.EXHAUST_EXCHANGE;
            case SCORE_SWITCH:
                mThresholdStart = Double.POSITIVE_INFINITY;
                mThresholdEnd = Double.POSITIVE_INFINITY;
                return SystemState.EXHAUST_SWITCH;
            case ACQUIRE_FLOOR:
                mThresholdStart = Double.POSITIVE_INFINITY;
                mThresholdEnd = Double.POSITIVE_INFINITY;
                return SystemState.INTAKE_FLOOR;
            case ACQUIRE_STACK:
                mThresholdStart = Double.POSITIVE_INFINITY;
                mThresholdEnd = Double.POSITIVE_INFINITY;
                return SystemState.INTAKE_STACK;
            default:
                mThresholdStart = Double.POSITIVE_INFINITY;
                return SystemState.STOWED;
        }
    }

    private SystemState handleStowing(double timeInState) {
        /*if (mArmTalon.getClosedLoopError(0) < kAllowableClosedLoopError && !mLimitSwitch.get()) {
            mLeftSpark.set(kStowingMotorSetpoint);
            mRightSpark.set(-kStowingMotorSetpoint);
            mArmTalon.set(ControlMode.Position, -200);
            return SystemState.STOWING;
        } else if (!mLimitSwitch.get()) {
            mLeftSpark.set(kStowingMotorSetpoint);
            mRightSpark.set(-kStowingMotorSetpoint);
            mArmTalon.set(ControlMode.Position, kHoldingSetpoint);
            return SystemState.STOWING;
        } else {
            mArmTalon.setSelectedSensorPosition(0, 0, 0);
            mArmTalon.set(ControlMode.Position, kHoldingSetpoint);
        }*/
        //mLeftSolenoid.set( DoubleSolenoid.Value.kForward);
        if (mLED.isSignaling()) {
            if (DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Red)
                mLED.setWantedState(LED.WantedState.ALLIANCE_RED);
            else
                mLED.setWantedState(LED.WantedState.ALLIANCE_BLUE);
        }
        if (mLimitSwitch.get()) {
            mArmTalon.set(ControlMode.PercentOutput, -1);
            mThresholdStart = Double.POSITIVE_INFINITY;
            mLeftSpark.set(kStowingMotorSetpoint);
            mRightSpark.set(-kStowingMotorSetpoint);
        } else {
            mArmTalon.setSelectedSensorPosition(0,0,0);
            mArmTalon.set(ControlMode.PercentOutput, 0);
            if (mThresholdStart == Double.POSITIVE_INFINITY) {
                mThresholdStart = timeInState;
            } else {
                if (timeInState - mThresholdStart > kStowingDelay) {
                    mLeftSpark.set(kStowingMotorSetpoint);
                    mRightSpark.set(-kStowingMotorSetpoint);
                } else {
                    return SystemState.STOWED;
                }
            }
        }
        //mArmTalon.set(ControlMode.Position, kHoldingSetpoint);
        //mLeftSolenoid.set( DoubleSolenoid.Value.kForward);
        //mRightSolenoid.set(DoubleSolenoid.Value.kForward);
        /*if (mArmTalon.getClosedLoopError(0) < kAllowableClosedLoopError) {
            if (mThresholdStart == Double.POSITIVE_INFINITY) {
                mThresholdStart = timeInState;
            } else {
                if (timeInState - mThresholdStart > kStowingDelay) {
                    mLeftSpark.set(kStowingMotorSetpoint);
                    mRightSpark.set(-kStowingMotorSetpoint);
                } else {
                    return SystemState.STOWED;
                }
            }
        } else {
            mThresholdStart = Double.POSITIVE_INFINITY;
            mLeftSpark.set(kStowingMotorSetpoint);
            mRightSpark.set(-kStowingMotorSetpoint);
        }*/
        return SystemState.STOWING;
    }

    private SystemState handleStowed(double timeInState) {
        //mLeftSolenoid.set( DoubleSolenoid.Value.kForward);
        //mRightSolenoid.set(DoubleSolenoid.Value.kForward);
        switch(mWantedState) {
            case ACQUIRE_FLOOR:
                mThresholdStart = Double.POSITIVE_INFINITY;
                mThresholdEnd = Double.POSITIVE_INFINITY;
                return SystemState.INTAKE_FLOOR;
            case ACQUIRE_STACK:
                mThresholdStart = Double.POSITIVE_INFINITY;
                mThresholdEnd = Double.POSITIVE_INFINITY;
                return SystemState.INTAKE_STACK;
            case SCORE_EXCHANGE:
                mThresholdStart = Double.POSITIVE_INFINITY;
                mThresholdEnd = Double.POSITIVE_INFINITY;
                return SystemState.EXHAUST_EXCHANGE;
            case SCORE_SWITCH:
                mThresholdStart = Double.POSITIVE_INFINITY;
                mThresholdEnd = Double.POSITIVE_INFINITY;
                return SystemState.EXHAUST_SWITCH;
            case LOAD_SHOOTER:
                mThresholdStart = Double.POSITIVE_INFINITY;
                mThresholdEnd = Double.POSITIVE_INFINITY;
                return SystemState.EXHAUST_SHOOTER;
            default:
                //mArmTalon.set(ControlMode.Position, kHoldingSetpoint);
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

        //SmartDashboard.putNumber("Intake Left Current", mPDP.getCurrent(kPDPSparkSlot));
        //SmartDashboard.putNumber("Intake Right Current", mPDP.getCurrent(6));
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
    double max = 0;
    public boolean checkSystem() {
        boolean failure = mLimitSwitch.get();


        return failure;
    }
}

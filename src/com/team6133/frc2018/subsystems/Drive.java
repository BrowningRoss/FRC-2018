package com.team6133.frc2018.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.team6133.frc2018.Constants;
import com.team6133.frc2018.loops.Loop;
import com.team6133.frc2018.loops.Looper;
import com.team6133.lib.util.DriveSignal;
import com.team6133.lib.util.Util;
import com.team6133.lib.util.drivers.CANTalonFactory;
import com.team6133.lib.util.drivers.NavXmicro;
import com.team6133.lib.util.math.Rotation2d;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;

/**
 * This subsystem consists of the robot's drivetrain: 4 CIM motors, 4 talons,
 * one solenoid and 2 pistons to shift gears, and a navX board. The Drive
 * subsystem has several control methods including open loop, velocity control,
 * and position control. The Drive subsystem also has several methods that
 * handle automatic aiming, autonomous path driving, and manual control.
 *
 * @see Subsystem.java
 */
public class Drive extends Subsystem {

    private static Drive mInstance = new Drive();
    // Mecanum Drive Controller
    private final MecanumDrive mMecanumDrive;
    // Hardware
    private final WPI_TalonSRX mFrontLeft, mFrontRight, mRearLeft, mRearRight;
    private final NavXmicro mNavXBoard;
    // Logging
    //???private final ReflectingCSVWriter<PathFollower.DebugOutput> mCSVWriter;
    // Control states
    private DriveControlState mDriveControlState;
    // These gains get reset below!!
    private Rotation2d mTargetHeading = new Rotation2d(Rotation2d.fromDegrees(0));
    // Loop control
    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized (Drive.this) {
                setOpenLoop(DriveSignal.NEUTRAL);
                mNavXBoard.reset();
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Drive.this) {
                switch (mDriveControlState) {
                    case OPEN_LOOP:
                        return;
                    case HEADING_SETPOINT:
                        return;
                    case TURN_TO_HEADING:
                        return;
                    case POLAR_DRIVE:
                        return;
                    default:
                        System.out.println("Unexpected drive control state: " + mDriveControlState);
                        break;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
            //---mCSVWriter.flush();
        }
    };

    private class PIDTwist extends PIDSubsystem {

        private DriveSignal mSignal;
        public PIDTwist(double kP, double kI, double kD, double period) {
            super(kP, kI, kD, period);
            setAbsoluteTolerance(1.5);
            getPIDController().setContinuous(true);
        }

        public void setDriveSignal(DriveSignal sig) {
            mSignal = sig;
        }

        @Override
        protected double returnPIDInput() {
            return getGyroAngle().getDegrees();
        }

        @Override
        protected void usePIDOutput(double output) {
            mMecanumDrive.driveCartesian(mSignal.getX(), mSignal.getY(), output, getGyroAngle().getDegrees());
        }

        @Override
        protected void initDefaultCommand() {

        }
    }

    private final PIDTwist mPIDTwist = new PIDTwist(0.0175, 0.0015, 0, 5);

    private boolean mIsOnTarget = false;
    private boolean mIsApproaching = false;
    private Drive() {
        // Start all Talons in open loop mode.
        mFrontLeft = CANTalonFactory.createDefaultTalon(Constants.kFrontLeftDriveId);
        mFrontLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 10);

        mFrontRight = CANTalonFactory.createDefaultTalon(Constants.kFrontRightDriveId);
        mFrontRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 10);

        mRearLeft = CANTalonFactory.createDefaultTalon(Constants.kRearLeftDriveId);
        mRearLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 10);

        mRearRight = CANTalonFactory.createDefaultTalon(Constants.kRearRightDriveId);
        mRearRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 10);

        mPIDTwist.disable();


        // Path Following stuff
        mNavXBoard = new NavXmicro(I2C.Port.kOnboard);

        // Initialize the Mecanum Drive
        mMecanumDrive = new MecanumDrive(mFrontLeft, mRearLeft, mFrontRight, mRearRight);
        setOpenLoop(DriveSignal.NEUTRAL);

    }

    public static Drive getInstance() {
        return mInstance;
    }

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }

    /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            mDriveControlState = DriveControlState.OPEN_LOOP;
            System.out.println("Starting open loop control.");
            mPIDTwist.disable();
        }
        try {
            mMecanumDrive.driveCartesian(signal.getX(), signal.getY(), signal.getTwist(), getGyroAngle().getDegrees());
        } catch (Throwable t) {
            mMecanumDrive.driveCartesian(signal.getX(), signal.getY(), signal.getTwist());
            throw t;
        }
    }

    public synchronized void setClosedLoop( DriveSignal signal, double heading) {
        if (mDriveControlState != DriveControlState.HEADING_SETPOINT) {
            mDriveControlState = DriveControlState.HEADING_SETPOINT;

            System.out.println("Starting closed loop control with heading = " + heading);
        }
        mTargetHeading = Rotation2d.fromDegrees(heading);
        //---double dx = mTargetHeading.getDegrees() - getGyroAngle().getDegrees();
        //---double kP = 0.0175;


        //---double pidTwist = DriveHelper.throttleTwist(kP * dx);

        try {
            mPIDTwist.setDriveSignal(signal);
            if (mPIDTwist.getSetpoint() != mTargetHeading.getDegrees()) {
                mPIDTwist.setSetpoint(mTargetHeading.getDegrees());
            }
            mPIDTwist.enable();
            //---mMecanumDrive.driveCartesian(signal.getX(), signal.getY(), -pidTwist, getGyroAngle().getDegrees());
        } catch (Throwable t) {
            mMecanumDrive.driveCartesian(signal.getX(), signal.getY(), signal.getTwist());
            throw t;
        }
    }

    public synchronized void setPolarDrive(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.POLAR_DRIVE) {
            mDriveControlState = DriveControlState.POLAR_DRIVE;
        }
        try {
            mMecanumDrive.drivePolar(signal.getY(), getGyroAngle().getDegrees(), signal.getTwist());
        } catch (Throwable t) {
            throw t;
        }
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("front left voltage (V)", mFrontLeft.getMotorOutputVoltage());
        SmartDashboard.putNumber("front right voltage (V)", mFrontRight.getMotorOutputVoltage());
        SmartDashboard.putNumber("rear left voltage (V)", mRearLeft.getMotorOutputVoltage());
        SmartDashboard.putNumber("rear right voltage (V)", mRearRight.getMotorOutputVoltage());

        SmartDashboard.putNumber("gyro vel", getGyroVelocityDegreesPerSec());
        SmartDashboard.putNumber("gyro deg", getGyroAngle().getDegrees());
    }

    @Override
    public void zeroSensors() {
        mNavXBoard.zeroYaw();
    }

    public synchronized Rotation2d getGyroAngle() {
        return mNavXBoard.getYaw();
    }

    public synchronized void setGyroAngle(Rotation2d angle) {
        mNavXBoard.reset();
        mNavXBoard.setAngleAdjustment(angle);
    }

    public synchronized NavXmicro getNavXBoard() {
        return mNavXBoard;
    }

    public synchronized double getGyroVelocityDegreesPerSec() {
        return mNavXBoard.getYawRateDegreesPerSec();
    }


    // The robot drivetrain's various states.
    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        HEADING_SETPOINT, // heading PID control
        TURN_TO_HEADING, // turn in place
        POLAR_DRIVE,
    }

    public boolean checkSystem() {
        System.out.println("Testing DRIVE.---------------------------------");
        final double kCurrentThres = 0.5;

        mFrontLeft.set(ControlMode.Current, 0);
        mRearLeft.set(ControlMode.Current, 0);
        mFrontRight.set(ControlMode.Current, 0);
        mRearRight.set(ControlMode.Current, 0);

        mFrontLeft.set(ControlMode.PercentOutput, 0.75);
        Timer.delay(4.0);
        final double currentFrontLeft = mFrontLeft.getOutputCurrent();
        mFrontLeft.set(ControlMode.PercentOutput, 0);

        Timer.delay(2.0);

        mRearLeft.set(ControlMode.PercentOutput, 0.75);
        Timer.delay(4.0);
        final double currentRearLeft = mRearLeft.getOutputCurrent();
        mRearLeft.set(ControlMode.PercentOutput, 0);

        Timer.delay(2.0);

        mFrontRight.set(ControlMode.PercentOutput, 0.75);
        Timer.delay(4.0);
        final double currentFrontRight = mFrontRight.getOutputCurrent();
        mFrontRight.set(ControlMode.PercentOutput, 0);

        Timer.delay(2.0);

        mRearRight.set(ControlMode.PercentOutput, 0.75);
        Timer.delay(4.0);
        final double currentRearRight = mRearRight.getOutputCurrent();
        mRearRight.set(ControlMode.PercentOutput, 0);


        System.out.println("Front Left  Current: " + currentFrontLeft  + " Rear Left  Current: "
                + currentRearLeft);
        System.out.println("Front Right Current: " + currentFrontRight + " Rear Right Current: "
                + currentRearRight);

        boolean failure = false;

        if (currentFrontRight < kCurrentThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Front Right Current Low !!!!!!!!!!");
        }

        if (currentRearRight < kCurrentThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Rear  Right Current Low !!!!!!!!!!");
        }

        if (currentFrontLeft < kCurrentThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Front Left  Current Low !!!!!!!!!!");
        }

        if (currentRearLeft < kCurrentThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Rear  Left  Current Low !!!!!!!!!!");
        }

        if (!Util.allCloseTo(Arrays.asList(currentFrontLeft, currentRearLeft, currentFrontRight, currentRearRight), currentFrontLeft,
                5.0)) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Drive Currents Different !!!!!!!!!!");
        }

        return !failure;
    }
}
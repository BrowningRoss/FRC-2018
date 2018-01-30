package com.team6133.frc2018.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.team6133.frc2018.Constants;
import com.team6133.frc2018.loops.Loop;
import com.team6133.frc2018.loops.Looper;
import com.team6133.lib.util.DriveSignal;
import com.team6133.lib.util.Util;
import com.team6133.lib.util.control.SynchronousPIDF;
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
    // PID for heading control
    private final SynchronousPIDF mPIDTwist = new SynchronousPIDF(Constants.kDriveTwistKp, Constants.kDriveTwistKi,
                                                Constants.kDriveTwistKd, Constants.kDriveTwistKf);
    // PID for proximity control
    private final SynchronousPIDF mPIDProxFront = new SynchronousPIDF(Constants.kDriveProxKp, Constants.kDriveProxKi,
                                                Constants.kDriveProxKd, Constants.kDriveProxKf);
    private final double mThresholdTime = 0.33;     // time to delay in LAUNCH_SETPOINT state before using proximity sensors.
    private double mTimeInState;
    private double mCurrentStateStartTime;
    // Loop control
    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized (Drive.this) {
                setOpenLoop(DriveSignal.NEUTRAL);
                mNavXBoard.reset();
            }
            mCurrentStateStartTime = Timer.getFPGATimestamp();
            mTimeInState = 0.0;
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Drive.this) {
                mTimeInState = Timer.getFPGATimestamp() - mCurrentStateStartTime;
                switch (mDriveControlState) {
                    case OPEN_LOOP:
                        return;
                    case HEADING_SETPOINT:
                        mPIDTwist.calculate(getGyroAngle().getDegrees(), Constants.kLooperDt);
                        return;
                    case LAUNCH_SETPOINT:
                        mPIDTwist.calculate(getGyroAngle().getDegrees(), Constants.kLooperDt);
                        mPIDProxFront.calculate(0.000, Constants.kLooperDt);    // @TODO: Add IRPD or Ultrasonic input
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

        mPIDTwist.setInputRange(-180.0, 180.0);
        mPIDTwist.setOutputRange(-Constants.kTwistMaxOutput, Constants.kTwistMaxOutput);
        mPIDTwist.setContinuous();
        mPIDTwist.setDeadband(0.02);

        mPIDProxFront.setDeadband(0.02);
        mPIDProxFront.setOutputRange(-1.0, 1.0);
        mPIDProxFront.setInputRange(0, 100); //@TODO: Determine the correct input range


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
     * Open loop control of the robot.
     * @param signal - the joystick drive signal
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            mDriveControlState = DriveControlState.OPEN_LOOP;
            System.out.println("Starting open loop control.");
            mCurrentStateStartTime = Timer.getFPGATimestamp();
            mPIDTwist.reset();
            mPIDProxFront.reset();
        }
        try {
            mMecanumDrive.driveCartesian(signal.getX(), signal.getY(), signal.getTwist(), getGyroAngle().getDegrees());
        } catch (Throwable t) {
            mMecanumDrive.driveCartesian(signal.getX(), signal.getY(), signal.getTwist());
            throw t;
        }
    }

    /**
     * Enables PID Heading Control.  The driver can still operate in the x & y directions, but this function
     * overrides the twist such that the robot strictly faces left or right.
     * Used for aiming.
     * @param signal - the joystick drive signal
     * @param heading - the PID setpoint
     */

    public synchronized void setClosedLoopHeading( DriveSignal signal, double heading) {
        mTargetHeading = Rotation2d.fromDegrees(heading);
        if (mPIDTwist.getSetpoint() != mTargetHeading.getDegrees()) {
            mPIDTwist.setSetpoint(mTargetHeading.getDegrees());
        }
        if (mDriveControlState != DriveControlState.HEADING_SETPOINT) {
            mDriveControlState = DriveControlState.HEADING_SETPOINT;
            mCurrentStateStartTime = Timer.getFPGATimestamp();
            mPIDTwist.resetIntegrator();
            System.out.println("Starting closed loop control with heading = " + heading);
        }

        //---double dx = mTargetHeading.getDegrees() - getGyroAngle().getDegrees();
        //---double kP = 0.0175;
        //---double pidTwist = DriveHelper.throttleTwist(kP * dx);

        try {

            mMecanumDrive.driveCartesian(signal.getX(), signal.getY(), mPIDTwist.get(), getGyroAngle().getDegrees());
        } catch (Throwable t) {
            mMecanumDrive.driveCartesian(signal.getX(), signal.getY(), signal.getTwist());
            throw t;
        }
    }

    /**
     * PID Control loop for aligning the drive for shooting.
     * The robot will automatically face left or right (depending on the location of the scale).
     * After a small delay, the robot will also use the proximity sensor to adjust to the GUARD RAIL.
     * @param signal - the joystick drive signal
     */
    public synchronized void handleAlignLaunch(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.LAUNCH_SETPOINT) {
            mPIDProxFront.resetIntegrator();
            mPIDTwist.resetIntegrator();
            if (Constants.kGameSpecificMessage.charAt(1) == 'L') {
                mTargetHeading = Rotation2d.fromDegrees(-90);
            } else {
                mTargetHeading = Rotation2d.fromDegrees(90);
            }
            mPIDTwist.setSetpoint(mTargetHeading.getDegrees());
            mPIDProxFront.setSetpoint(Constants.kLaunchProxSetpoint);
            mDriveControlState = DriveControlState.LAUNCH_SETPOINT;
            mCurrentStateStartTime = Timer.getFPGATimestamp();
            System.out.println("Aligning drive for " + Constants.kGameSpecificMessage.charAt(1) + " scale shot.");
        }
        if (mTimeInState > mThresholdTime) {
            mMecanumDrive.driveCartesian(mPIDProxFront.get(), signal.getY(), mPIDTwist.get(), getGyroAngle().getDegrees());
        } else {
            mMecanumDrive.driveCartesian(signal.getX(), signal.getY(), mPIDTwist.get(), getGyroAngle().getDegrees());
        }
    }

    /**
     * Configure the robot for Polar Drive.  Not likely to be used, but written for testing purposes.
     * @param signal - the joystick drive signal
     */
    public synchronized void setPolarDrive(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.POLAR_DRIVE) {
            mDriveControlState = DriveControlState.POLAR_DRIVE;
            mCurrentStateStartTime = Timer.getFPGATimestamp();
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
        OPEN_LOOP,          // open loop voltage control
        HEADING_SETPOINT,   // heading PID control
        LAUNCH_SETPOINT,    // launch PID control
        POLAR_DRIVE,        // not used - here just for testing
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
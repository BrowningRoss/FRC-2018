package com.team6133.frc2018.subsystems;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.team6133.frc2018.Constants;
import com.team6133.frc2018.loops.Loop;
import com.team6133.frc2018.loops.Looper;
import com.team6133.lib.util.DriveSignal;
import com.team6133.lib.util.ReflectingCSVWriter;
import com.team6133.lib.util.control.PathFollower;
import com.team6133.lib.util.drivers.CANTalonFactory;
import com.team6133.lib.util.drivers.NavXmicro;
import com.team6133.lib.util.math.Rotation2d;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    private final ReflectingCSVWriter<PathFollower.DebugOutput> mCSVWriter;
    // Control states
    private DriveControlState mDriveControlState;
    // These gains get reset below!!
    private Rotation2d mTargetHeading = new Rotation2d();
    // Hardware states
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
                    default:
                        System.out.println("Unexpected drive control state: " + mDriveControlState);
                        break;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
            mCSVWriter.flush();
        }
    };
    private boolean mIsOnTarget = false;
    private boolean mIsApproaching = false;
    private Drive() {
        // Start all Talons in open loop mode.
        mFrontLeft = CANTalonFactory.createDefaultTalon(Constants.kFrontLeftDriveId);
        mFrontLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);

        mFrontRight = CANTalonFactory.createDefaultTalon(Constants.kFrontRightDriveId);
        mFrontRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);

        mRearLeft = CANTalonFactory.createDefaultTalon(Constants.kRearLeftDriveId);
        mRearLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);

        mRearRight = CANTalonFactory.createDefaultTalon(Constants.kRearRightDriveId);
        mRearRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);




        // Path Following stuff
        mNavXBoard = new NavXmicro(I2C.Port.kOnboard);

        // Initialize the Mecanum Drive
        mMecanumDrive = new MecanumDrive(mFrontLeft, mRearLeft, mFrontRight, mRearRight);
        setOpenLoop(DriveSignal.NEUTRAL);

        mCSVWriter = new ReflectingCSVWriter<PathFollower.DebugOutput>("/home/lvuser/PATH-FOLLOWER-LOGS.csv",
                PathFollower.DebugOutput.class);
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
        }
        try {
            mMecanumDrive.driveCartesian(signal.getX(), signal.getY(), signal.getTwist(), getGyroAngle().getDegrees());
        } catch (NullPointerException ex) {
            mMecanumDrive.driveCartesian(signal.getX(), signal.getY(), signal.getTwist());
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
    }


}
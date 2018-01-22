package com.team6133.frc2018.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

import com.team6133.frc2018.Constants;
import com.team6133.frc2018.Kinematics;
import com.team6133.frc2018.RobotState;
import com.team6133.frc2018.loops.Loop;
import com.team6133.frc2018.loops.Looper;
import com.team6133.lib.util.DriveSignal;
import com.team6133.lib.util.ReflectingCSVWriter;
import com.team6133.lib.util.Util;
import com.team6133.lib.util.control.Lookahead;
import com.team6133.lib.util.control.Path;
import com.team6133.lib.util.control.PathFollower;
import com.team6133.lib.util.drivers.CANTalonFactory;
import com.team6133.lib.util.drivers.NavX;
import com.team6133.lib.util.math.RigidTransform2d;
import com.team6133.lib.util.math.Rotation2d;
import com.team6133.lib.util.math.Twist2d;

import java.util.Arrays;
import java.util.Optional;

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

	public static Drive getInstance() {
		return mInstance;
	}

	// The robot drivetrain's various states.
	public enum DriveControlState {
		OPEN_LOOP, // open loop voltage control
		HEADING_SETPOINT, // heading PID control
		TURN_TO_HEADING, // turn in place
	}

	/**
	 * Check if the drive talons are configured for velocity control
	 */
	protected static boolean usesTalonVelocityControl(DriveControlState state) {
		if (state == DriveControlState.VELOCITY_SETPOINT || state == DriveControlState.PATH_FOLLOWING) {
			return true;
		}
		return false;
	}

	/**
	 * Check if the drive talons are configured for position control
	 */
	protected static boolean usesTalonPositionControl(DriveControlState state) {
		return false;
	}

	// Mecanum Drive Controller
	private final MecanumDrive mMecanumDrive;

	// Control states
	private DriveControlState mDriveControlState;

	// Hardware
	private final WPI_TalonSRX mFrontLeft, mFrontRight, mRearLeft, mRearRight;
	private final NavX mNavXBoard;

	// Controllers
	private RobotState mRobotState = RobotState.getInstance();
	private PathFollower mPathFollower;

	// These gains get reset below!!
	private Rotation2d mTargetHeading = new Rotation2d();
	private Path mCurrentPath = null;

	// Hardware states
	private boolean mIsBrakeMode;
	private boolean mIsOnTarget = false;
	private boolean mIsApproaching = false;

	// Logging
	private final ReflectingCSVWriter<PathFollower.DebugOutput> mCSVWriter;

	private final Loop mLoop = new Loop() {
		@Override
		public void onStart(double timestamp) {
			synchronized (Drive.this) {
				setOpenLoop(DriveSignal.NEUTRAL);
				setBrakeMode(false);
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


		setOpenLoop(DriveSignal.NEUTRAL);

		// Path Following stuff
		mNavXBoard = new NavX(SerialPort.Port.kUSB);

		// Force a CAN message across.
		mIsBrakeMode = true;
		setBrakeMode(false);

		// Initialize the Mecanum Drive
		mMecanumDrive = new MecanumDrive(mFrontLeft, mRearLeft, mFrontRight, mRearRight);

		mCSVWriter = new ReflectingCSVWriter<PathFollower.DebugOutput>("/home/lvuser/PATH-FOLLOWER-LOGS.csv",
				PathFollower.DebugOutput.class);
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
			mFrontLeft.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
			mFrontRight.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
			mFrontLeft.configNominalOutputVoltage(0.0, 0.0);
			mFrontRight.configNominalOutputVoltage(0.0, 0.0);

			mRearLeft.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
			mRearRight.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
			mRearLeft.configNominalOutputVoltage(0.0, 0.0);
			mRearRight.configNominalOutputVoltage(0.0, 0.0);
			mDriveControlState = DriveControlState.OPEN_LOOP;
			setBrakeMode(false);
		}
		// Right side is reversed, but reverseOutput doesn't invert PercentVBus.
		// So set negative on the right master.


	}

	public boolean isBrakeMode() {
		return mIsBrakeMode;
	}

	public synchronized void setBrakeMode(boolean on) {
		if (mIsBrakeMode != on) {
			mIsBrakeMode = on;
			mFrontLeft.enableBrakeMode(on);
			mFrontRight.enableBrakeMode(on);
			mRearLeft.enableBrakeMode(on);
			mRearRight.enableBrakeMode(on);
		}
	}

	@Override
	public synchronized void stop() {
		setOpenLoop(DriveSignal.NEUTRAL);
	}

	@Override
	public void outputToSmartDashboard() {
		SmartDashboard.putNumber("front left voltage (V)", mFrontLeft.getOutputVoltage());
		SmartDashboard.putNumber("front right voltage (V)", mFrontRight.getOutputVoltage());
		SmartDashboard.putNumber("rear left voltage (V)", mRearLeft.getOutputVoltage());
		SmartDashboard.putNumber("rear right voltage (V)", mRearRight.getOutputVoltage());

		synchronized (this) {
			if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
				SmartDashboard.putNumber("drive CTE", mPathFollower.getCrossTrackError());
				SmartDashboard.putNumber("drive ATE", mPathFollower.getAlongTrackError());
			} else {
				SmartDashboard.putNumber("drive CTE", 0.0);
				SmartDashboard.putNumber("drive ATE", 0.0);
			}
		}
		SmartDashboard.putNumber("gyro vel", getGyroVelocityDegreesPerSec());
		SmartDashboard.putNumber("gyro pos", getGyroAngle().getDegrees());
	}

	@Override
	public void zeroSensors() {
		mNavXBoard.zeroYaw();
	}


	private static double rotationsToInches(double rotations) {
		return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
	}

	private static double rpmToInchesPerSecond(double rpm) {
		return rotationsToInches(rpm) / 60;
	}

	private static double inchesToRotations(double inches) {
		return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
	}

	private static double inchesPerSecondToRpm(double inches_per_second) {
		return inchesToRotations(inches_per_second) * 60;
	}

	public synchronized Rotation2d getGyroAngle() {
		return mNavXBoard.getYaw();
	}

	public synchronized NavX getNavXBoard() {
		return mNavXBoard;
	}

	public synchronized void setGyroAngle(Rotation2d angle) {
		mNavXBoard.reset();
		mNavXBoard.setAngleAdjustment(angle);
	}

	public synchronized double getGyroVelocityDegreesPerSec() {
		return mNavXBoard.getYawRateDegreesPerSec();
	}


}
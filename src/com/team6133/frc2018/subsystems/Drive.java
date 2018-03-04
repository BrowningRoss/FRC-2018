package com.team6133.frc2018.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.team6133.frc2018.Constants;
import com.team6133.frc2018.auto.AutonPathSettings;
import com.team6133.frc2018.loops.Loop;
import com.team6133.frc2018.loops.Looper;
import com.team6133.lib.util.DriveSignal;
import com.team6133.lib.util.SensorTarget;
import com.team6133.lib.util.TimeDelayedBoolean;
import com.team6133.lib.util.Util;
import com.team6133.lib.util.control.SynchronousPIDF;
import com.team6133.lib.util.drivers.CANTalonFactory;
import com.team6133.lib.util.drivers.IRSensor;
import com.team6133.lib.util.drivers.NavXmicro;
import com.team6133.lib.util.drivers.UltrasonicSensor;
import com.team6133.lib.util.math.Rotation2d;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;

/**
 * This subsystem consists of the robot's drivetrain: 4 CIM motors, 4 talons,
 * and a navXmicro board. The Drive subsystem has several control methods
 * including open loop, heading control, and proximity control. The Drive
 * subsystem also has several methods that handle automatic aiming alignment, autonomous path driving,
 * and manual control.
 *
 * @see Subsystem
 */
public class Drive extends Subsystem {

    private static Drive mInstance = new Drive();
    // Mecanum Drive Controller
    private final MecanumDrive mMecanumDrive;
    // Hardware
    private final WPI_TalonSRX mFrontLeft, mFrontRight, mRearLeft, mRearRight;
    private NavXmicro mNavXBoard;
    public final UltrasonicSensor mFrontSensor;
    public IRSensor mLeftSensor;
    public IRSensor mRightSensor;
    // Logging
    //???private final ReflectingCSVWriter<PathFollower.DebugOutput> mCSVWriter;
    // Control states
    private DriveControlState mDriveControlState;
    private DriveSignal mDriveSignal;
    private AutonPathSettings mPathSetting;
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
                setOpenLoop();
                if (Constants.Robot_Auton_Start_Time < 15)
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
                        mFrontSensor.update();
                        mPIDTwist.calculate(getGyroAngle().getDegrees(), Constants.kLooperDt);
                        mPIDProxFront.calculate(mFrontSensor.getAverageDistance(), Constants.kLooperDt);
                        return;
                    case POLAR_DRIVE:
                        return;
                    case AUTON_PATH:
                        mFrontSensor.update();
                        mPIDTwist.calculate(getGyroAngle().getDegrees(), Constants.kLooperDt);
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

    private TimeDelayedBoolean mAutonTimedBoolean;
    private Drive() {
        // Start all Talons in open loop mode.
        mFrontLeft = CANTalonFactory.createDefaultTalon(Constants.kFrontLeftDriveId);
        mFrontLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 5, 0);
        mFrontLeft.configPeakCurrentLimit(Constants.Drive_Current_Limit, 0);
        mFrontLeft.configContinuousCurrentLimit(Constants.Drive_Continuous_Current_Limit, 0);
        mFrontLeft.configPeakCurrentDuration(Constants.Drive_Current_Timeout_Ms, 0);

        mFrontRight = CANTalonFactory.createDefaultTalon(Constants.kFrontRightDriveId);
        mFrontRight.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 5, 0);
        mFrontRight.configPeakCurrentLimit(Constants.Drive_Current_Limit, 0);
        mFrontRight.configContinuousCurrentLimit(Constants.Drive_Continuous_Current_Limit, 0);
        mFrontRight.configPeakCurrentDuration(Constants.Drive_Current_Timeout_Ms, 0);

        mRearLeft = CANTalonFactory.createDefaultTalon(Constants.kRearLeftDriveId);
        mRearLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 5, 0);
        mRearLeft.configPeakCurrentLimit(Constants.Drive_Current_Limit, 0);
        mRearLeft.configContinuousCurrentLimit(Constants.Drive_Continuous_Current_Limit, 0);
        mRearLeft.configPeakCurrentDuration(Constants.Drive_Current_Timeout_Ms, 0);

        mRearRight = CANTalonFactory.createDefaultTalon(Constants.kRearRightDriveId);
        mRearRight.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 5, 0);
        mRearRight.configPeakCurrentLimit(Constants.Drive_Current_Limit, 0);
        mRearRight.configContinuousCurrentLimit(Constants.Drive_Continuous_Current_Limit, 0);
        mRearRight.configPeakCurrentDuration(Constants.Drive_Current_Timeout_Ms, 0);

        mPIDTwist.setInputRange(-180.0, 180.0);
        mPIDTwist.setOutputRange(-Constants.kTwistMaxOutput, Constants.kTwistMaxOutput);
        mPIDTwist.setContinuous();
        mPIDTwist.setDeadband(.25);



        mPIDProxFront.setDeadband(1.5);
        mPIDProxFront.setOutputRange(-1.0, 1.0);
        mPIDProxFront.setInputRange(0.8, 158);

        mFrontSensor = new UltrasonicSensor(10,12);
        mRightSensor = new IRSensor(Constants.kIRPDRightPort, Constants.MIN_TRIGGER_VOLTAGE, Constants.MAX_TRIGGER_VOLTAGE);
        mLeftSensor  = new IRSensor(Constants.kIRPDLeftPort, Constants.MIN_TRIGGER_VOLTAGE, Constants.MAX_TRIGGER_VOLTAGE);


        // NavXmicro using I2C on the RoboRIO (NOT using the MXP slot)
        mNavXBoard = new NavXmicro(I2C.Port.kOnboard);

        // Initialize the Mecanum Drive
        mMecanumDrive = new MecanumDrive(mFrontLeft, mRearLeft, mFrontRight, mRearRight);
        mMecanumDrive.setSafetyEnabled(false);
        mDriveSignal = DriveSignal.NEUTRAL;
        mAutonTimedBoolean = new TimeDelayedBoolean();
        setOpenLoop();

    }

    /**
     * Get the static instance of the Drive class
     * @return mInstance - the static instance of this class.
     */
    public static Drive getInstance() {
        return mInstance;
    }

    /**
     * Retrieves the current drive signal (joystick) values.
     * @return mDriveSignal - the most current values read from the joystick.
     */
    public DriveSignal getDriveSignal() { return mDriveSignal;}

    /**
     * Update the values of the drive signal.  Call this at the start of the teleop periodic loop.
     * @param signal The joystick drive signal.  Best practice is to apply deadband to the signal before passing it here.
     * @see com.team6133.lib.util.DriveHelper#mecDrive(double, double, double)
     */
    public synchronized void updateDriveSignal(DriveSignal signal) {
        mDriveSignal = signal;
    }

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }

    /**
     * Open loop control of the robot.
     * Call updateDriveSignal at the start of the teleop periodic loop.
     * This method relies on the mDriveSignal being updated in this way.
     */
    public synchronized void setOpenLoop() {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            mDriveControlState = DriveControlState.OPEN_LOOP;
            System.out.println("Starting open loop control.");
            mPIDTwist.reset();
            mPIDProxFront.reset();
        }
        try {
            mMecanumDrive.driveCartesian(mDriveSignal.getX(), mDriveSignal.getY(), mDriveSignal.getTwist(), getGyroAngle().getDegrees());
        } catch (Throwable t) {
            mMecanumDrive.driveCartesian(mDriveSignal.getX(), mDriveSignal.getY(), mDriveSignal.getTwist());
            throw t;
        }
    }

    /**
     * Enable the Heading Setpoint mode in teleop. Specify the heading and the method will prep the PID Controller.
     * The Drive State will change to HEADING_SETPOINT.
     * Call this method once in teleop periodic to initialize the mode, but then call updateTeleopHeadingSetpoint()
     * to allow the driver to update the drive signal.
     * @param heading Target heading to use with gyro. Input ranges from -180 to 180.
     * @param timeStamp The system start time that this method was called.
     */
    public synchronized void setTeleopHeadingSetpoint(double heading, double timeStamp) {
        Rotation2d heading_ = Rotation2d.fromDegrees(heading);
        if (Math.abs(heading_.inverse().rotateBy(mTargetHeading).getDegrees()) > 1E-3) {
            mTargetHeading = heading_;
        }
        if (mDriveControlState != DriveControlState.HEADING_SETPOINT) {
            mDriveControlState = DriveControlState.HEADING_SETPOINT;
            mTimeInState = 0.0;
            mPIDTwist.resetIntegrator();
            System.out.println("Starting closed loop control with heading = " + heading);
        }
        mPIDTwist.setSetpoint(mTargetHeading.getDegrees());
        mCurrentStateStartTime = timeStamp;
    }

    /**
     * Updates the PID Heading Control.  The driver can still operate in the x and y directions, but this function
     * overrides the twist such that the robot strictly faces left or right.
     * Useful for aiming the robot at a switch or scale.
     */
    public void updateTeleopHeadingSetpoint() {
        //---double dx = mTargetHeading.getDegrees() - getGyroAngle().getDegrees();
        //---double kP = 0.0175;
        //---double pidTwist = DriveHelper.throttleTwist(kP * dx);

        try {
            mMecanumDrive.driveCartesian(mDriveSignal.getX(), mDriveSignal.getY(), -mPIDTwist.get(), getGyroAngle().getDegrees());
        } catch (Throwable t) {
            mMecanumDrive.driveCartesian(mDriveSignal.getX(), mDriveSignal.getY(), mDriveSignal.getTwist());
            throw t;
        }
    }

    /**
     * PID Control loop for aligning the drive for shooting.
     * The robot will automatically face left or right (depending on the location of the scale).
     * After a small delay, the robot will also use the proximity sensor to adjust to the GUARD RAIL.
     * @param timeStamp The system timestamp when this method was called.
     */
    public synchronized void setAlignLaunch(double timeStamp) {
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
            mCurrentStateStartTime = timeStamp;
            mTimeInState = 0.0;
            System.out.println("Aligning drive for " + Constants.kGameSpecificMessage.charAt(1) + " scale shot.");
        }
    }

    /**
     * Uses PID sensors to fix the angle w.r.t the scale and the distance w.r.t the GUARD RAIL.
     * There is a small delay before the proximity sensor is used.  This is to compensate for the amount of time it
     * takes to orient the sensor facing the GUARD RAIL.
     * Call this method in teleop periodic after the initial call to setAlignLaunch()
     * @see Drive#setAlignLaunch(double)
     */
    public void updateAlignLaunch() {
        try {
            if (mTimeInState > mThresholdTime) {
                mMecanumDrive.driveCartesian(mPIDProxFront.get(), mDriveSignal.getY(), -mPIDTwist.get(), getGyroAngle().getDegrees());
            } else {
                mMecanumDrive.driveCartesian(mDriveSignal.getX(), mDriveSignal.getY(), -mPIDTwist.get(), getGyroAngle().getDegrees());
            }
        } catch (Throwable t) {
            mMecanumDrive.driveCartesian(mDriveSignal.getX(), mDriveSignal.getY(), mDriveSignal.getTwist());
            throw t;
        }
    }

    /**
     * Configure the robot for Polar Drive.  Not likely to be used, but written for testing purposes.
     * @param heading The target heading.  Input ranges from -180 to 180.
     * @param timeStamp The system time that this method was called.
     */
    public synchronized void setPolarDrive(double heading, double timeStamp) {
        if (mDriveControlState != DriveControlState.POLAR_DRIVE) {
            mDriveControlState = DriveControlState.POLAR_DRIVE;
            mCurrentStateStartTime = timeStamp;
            mTimeInState = 0.0;
        }
        try {
            mMecanumDrive.drivePolar(mDriveSignal.getY(), heading, mDriveSignal.getTwist());
        } catch (Throwable t) {
            throw t;
        }
        Timer.delay(0.005);
    }

    public synchronized void setAutonPath(AutonPathSettings pathSetting, double timeStamp) {
        mTargetHeading         = pathSetting.getHeading();
        mPathSetting           = pathSetting;
        mDriveControlState     = DriveControlState.AUTON_PATH;
        mCurrentStateStartTime = timeStamp;
        mTimeInState           = 0.0;

        mPIDTwist.resetIntegrator();
        mPIDTwist.setSetpoint(mTargetHeading.getDegrees());

        // Force a reset of the time delayed boolean
        mAutonTimedBoolean.update(false, mPathSetting.getTimeout());
        mAutonTimedBoolean.update(true, mPathSetting.getTimeout());
    }

    /**
     * Update the autonomous path
     * @return True if the sensor triggers true and the timer has passed
     */
    public boolean updateAutonPath() {
        boolean sensor;
        double magX = mPathSetting.getMagnitudeX(), magY = mPathSetting.getMagnitudeY();
        if (Math.abs(mPIDTwist.get()) > .2) {
            magX = 0;
            magY = 0;
        }
        if (mPathSetting.getSensorTarget().sensor == SensorTarget.Sensor.LeftIRPD) {
            sensor = mPathSetting.getInvertIRPD() != mLeftSensor.seesWall();
            //mMecanumDrive.driveCartesian(mPathSetting.getMagnitudeX(), mPathSetting.getMagnitudeY(), mPIDTwist.get(), getGyroAngle().getDegrees());
            mMecanumDrive.driveCartesian(magX, magY, -mPIDTwist.get(), getGyroAngle().getDegrees());
        } else if (mPathSetting.getSensorTarget().sensor == SensorTarget.Sensor.RightIRPD) {
            sensor = mRightSensor.seesWall() != mPathSetting.getInvertIRPD();
            mMecanumDrive.driveCartesian(magX, magY, -mPIDTwist.get(), getGyroAngle().getDegrees());
        } else if (mPathSetting.getSensorTarget().sensor == SensorTarget.Sensor.Ultra) {
            sensor = Math.abs(mFrontSensor.getAverageDistance() - mPathSetting.getSensorTarget().target) < 2;
            System.out.println("Ultra Distance:\t" + mFrontSensor.getAverageDistance());
            if (Math.abs(mFrontSensor.getAverageDistance() - mPathSetting.getSensorTarget().target) < 15) {
                // If we are < 15" from the target, slow down to 50% magnitude
                mMecanumDrive.driveCartesian(0.5 * magX, 0.5* magY, -mPIDTwist.get(), getGyroAngle().getDegrees());
            } else {
                mMecanumDrive.driveCartesian(magX, magY, -mPIDTwist.get(), getGyroAngle().getDegrees());
            }
        } else {
            sensor = mPIDTwist.onTarget(1.5);
            mMecanumDrive.driveCartesian(0,0, -mPIDTwist.get(), getGyroAngle().getDegrees());
        }
        //System.out.println("Sensor: " + sensor);
        return mAutonTimedBoolean.update(true, mPathSetting.getTimeout()) && sensor;
    }

    @Override
    public synchronized void stop() {
        mDriveSignal = DriveSignal.NEUTRAL;
        setOpenLoop();
    }

    @Override
    public void outputToSmartDashboard() {
        //SmartDashboard.putNumber("front left voltage (V)", mFrontLeft.getMotorOutputVoltage());
        //SmartDashboard.putNumber("front right voltage (V)", mFrontRight.getMotorOutputVoltage());
        //SmartDashboard.putNumber("rear left voltage (V)", mRearLeft.getMotorOutputVoltage());
        //SmartDashboard.putNumber("rear right voltage (V)", mRearRight.getMotorOutputVoltage());
        SmartDashboard.putNumber("Ultrasonic", mFrontSensor.getAverageDistance());
        SmartDashboard.putBoolean("Left IRPD", mLeftSensor.seesWall());
        SmartDashboard.putBoolean("Right IRPD", mRightSensor.seesWall());
        //SmartDashboard.putNumber("gyro vel", getGyroVelocityDegreesPerSec());
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

    public synchronized double getTargetHeading() {
        return mTargetHeading.getDegrees();
    }

    public void setVoltageLeftIRPD(double range) {
        double inverse_cm = 0.3937 / range;
        double voltage_min = 1.125 + 137.5 * inverse_cm;
        inverse_cm = 0.3937 / (range*.95);
        double voltage_max = 1.125 + 137.5 * inverse_cm;
        mLeftSensor.setLimitsVoltage(voltage_min, voltage_max);
    }

    public void setVoltageRightIRPD(double range) {
        double inverse_cm = 0.3937 / range;
        double voltage_min = 1.125 + 137.5 * inverse_cm;
        inverse_cm = 0.3937 / (range*.95);
        double voltage_max = 1.125 + 137.5 * inverse_cm;
        mRightSensor.setLimitsVoltage(voltage_min, voltage_max);
    }

    public void setVoltageBothIRPD(double range) {
        setVoltageLeftIRPD(range);
        setVoltageRightIRPD(range);
    }


    // The robot drivetrain's various states.
    public enum DriveControlState {
        OPEN_LOOP,          // open loop voltage control
        HEADING_SETPOINT,   // heading PID control
        LAUNCH_SETPOINT,    // launch PID control
        POLAR_DRIVE,        // not used - here just for testing
        AUTON_PATH,
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
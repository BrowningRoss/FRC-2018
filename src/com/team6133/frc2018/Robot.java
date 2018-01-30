package com.team6133.frc2018;

import com.team6133.frc2018.auto.AutoModeExecuter;
import com.team6133.frc2018.loops.Looper;
import com.team6133.frc2018.subsystems.ConnectionMonitor;
import com.team6133.frc2018.subsystems.Drive;
import com.team6133.frc2018.subsystems.Intake;
import com.team6133.lib.util.*;
import com.team6133.lib.util.drivers.RevDigitBoard;
import com.team6133.lib.util.math.RigidTransform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;

import java.util.Arrays;
import java.util.concurrent.ThreadLocalRandom;

/**
 * The main robot class, which instantiates all robot parts and helper classes
 * and initializes all loops. Some classes are already instantiated upon robot
 * startup; for those classes, the robot gets the instance as opposed to
 * creating a new object
 * <p>
 * After initializing all robot parts, the code sets up the autonomous and
 * teleoperated cycles and also code that runs periodically inside both
 * routines.
 * <p>
 * This is the nexus/converging point of the robot code and the best place to
 * start exploring.
 * <p>
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
    // Create subsystem manager
    private final SubsystemManager mSubsystemManager = new SubsystemManager(Arrays.asList(Drive.getInstance(),
            Intake.getInstance(),
            // ~!@Superstructure.getInstance(), Shooter.getInstance(),
            // Feeder.getInstance(), Hopper.getInstance(), Intake.getInstance(),
            ConnectionMonitor.getInstance()// , LED.getInstance(),
    ));
    // Get subsystem instances
    private Drive mDrive = Drive.getInstance();
    private Intake mIntake = Intake.getInstance();
    // ~!@private Superstructure mSuperstructure = Superstructure.getInstance();

    // private LED mLED = LED.getInstance();
    private AutoModeExecuter mAutoModeExecuter = null;
    // Initialize other helper objects
    private DriveHelper mDriveHelper = new DriveHelper();
    private ControlBoardInterface mControlBoard = ControlBoard.getInstance();

    private Looper mEnabledLooper = new Looper();

    private RevDigitBoard mRevDigitBoard = new RevDigitBoard();

    // ~!@private VisionServer mVisionServer = VisionServer.getInstance();

    // private AnalogInput mCheckLightButton = new
    // AnalogInput(Constants.kLEDOnId);

    private DelayedBoolean mDelayedAimButton;

    private LatchedBoolean mCommitTuning = new LatchedBoolean();
    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mTuningFlywheelMap = new InterpolatingTreeMap<>();

    public Robot() {
        CrashTracker.logRobotConstruction();
    }

    public void zeroAllSensors() {
        mSubsystemManager.zeroSensors();
        mDrive.zeroSensors();
    }

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            //~!@mEnabledLooper.register(VisionProcessor.getInstance());

            // mVisionServer.addVisionUpdateReceiver(VisionProcessor.getInstance());

            AutoModeSelector.initAutoModeSelector();

            mDelayedAimButton = new DelayedBoolean(Timer.getFPGATimestamp(), 0.1);
            // Force an true update now to prevent robot from running at start.
            mDelayedAimButton.update(Timer.getFPGATimestamp(), true);

            // Pre calculate the paths we use for auto.
            //~!@PathAdapter.calculatePaths();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
        zeroAllSensors();
    }

    /**
     * Initializes the robot for the beginning of autonomous mode (set
     * drivebase, intake and superstructure to correct states). Then gets the
     * correct auto mode from the AutoModeSelector
     *
     * @see AutoModeSelector.java
     */
    @Override
    public void autonomousInit() {
        try {
            CrashTracker.logAutoInit();

            System.out.println("Auto start timestamp: " + Timer.getFPGATimestamp());

            if (mAutoModeExecuter != null) {
                mAutoModeExecuter.stop();
            }

            zeroAllSensors();
            mIntake.setWantedState(Intake.WantedState.HOLDING);

            // ~!@mSuperstructure.setWantedState(Superstructure.WantedState.IDLE);
            // ~!@mSuperstructure.setActuateHopper(false);
            // ~!@mSuperstructure.setOverrideCompressor(true);

            mAutoModeExecuter = null;

            // ~!@Intake.getInstance().reset();

            // Shift to high
            // ~!@mDrive.setHighGear(true);
            // ~!@mDrive.setBrakeMode(true);

            mEnabledLooper.start();
            // ~!@mSuperstructure.reloadConstants();
            mAutoModeExecuter = new AutoModeExecuter();
            mAutoModeExecuter.setAutoMode(AutoModeSelector.getSelectedAutoMode());
            mAutoModeExecuter.start();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
        allPeriodic();
    }

    /**
     * Initializes the robot for the beginning of teleop
     */
    @Override
    public void teleopInit() {
        try {
            CrashTracker.logTeleopInit();

            // Start loopers
            mEnabledLooper.start();
            mDrive.setOpenLoop(DriveSignal.NEUTRAL);

            zeroAllSensors();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * This function is called periodically during operator control.
     * <p>
     * The code uses state machines to ensure that no matter what buttons the
     * driver presses, the robot behaves in a safe and consistent manner.
     * <p>
     * Based on driver input, the code sets a desired state for each subsystem.
     * Each subsystem will constantly compare its desired and actual states and
     * act to bring the two closer.
     */
    @Override
    public void teleopPeriodic() {
        try {
            double timestamp = Timer.getFPGATimestamp();
            // Drive base
            double xSpeed = mControlBoard.getThrottleX();
            double ySpeed = mControlBoard.getThrottleY();
            double twist  = mControlBoard.getTwist();
            boolean rotateLeft = mControlBoard.getRotateLeft();
            boolean rotateRight = mControlBoard.getRotateRight();


            // boolean wants_aim_button = mControlBoard.getAimButton();
            // wants_aim_button = !mDelayedAimButton.update(timestamp,
            // !wants_aim_button);
            if (!rotateLeft && !rotateRight) {
                mDrive.setOpenLoop(mDriveHelper.mecDrive(xSpeed, ySpeed, twist));
            } else if (rotateLeft) {
                mDrive.setClosedLoop(mDriveHelper.mecDrive(xSpeed, ySpeed, twist), 90);
            } else {
                mDrive.setClosedLoop(mDriveHelper.mecDrive(xSpeed, ySpeed, twist), -90);
            }

            /*
             * ~!@ if (wants_aim_button || mControlBoard.getDriveAimButton()) {
             *
             * if (Constants.kIsShooterTuning) { mDrive.setWantAimToGoal(); }
             * else if (mControlBoard.getDriveAimButton()) {
             * mDrive.setWantDriveTowardsGoal(); } else {
             * mDrive.setWantAimToGoal(); }
             *
             * if ((mControlBoard.getDriveAimButton() &&
             * !mDrive.isApproaching()) || !mControlBoard.getDriveAimButton()) {
             * if (mControlBoard.getUnjamButton()) {
             * mSuperstructure.setWantedState(Superstructure.WantedState.
             * UNJAM_SHOOT); } else {
             * mSuperstructure.setWantedState(Superstructure.WantedState.SHOOT);
             * } } else {
             * mSuperstructure.setWantedState(Superstructure.WantedState.
             * RANGE_FINDING); }
             *
             * } else {
             *
             * // Make sure not to interrupt shooting spindown. if
             * (!mSuperstructure.isShooting()) {
             * mDrive.setOpenLoop(mDriveHelper.cheesyDrive(throttle, turn,
             * mControlBoard.getQuickTurn(), !mControlBoard.getLowGear()));
             * boolean wantLowGear = mControlBoard.getLowGear();
             * mDrive.setHighGear(!wantLowGear); }
             *
             * Intake.getInstance().setCurrentThrottle(mControlBoard.getThrottle
             * ());
             *
             * boolean wantsExhaust = mControlBoard.getExhaustButton();
             *
             * if (Constants.kIsShooterTuning) {
             * mLED.setWantedState(LED.WantedState.FIND_RANGE); if
             * (mCommitTuning.update(mControlBoard.getLowGear())) { // Commit to
             * TuningMap. double rpm = mSuperstructure.getCurrentTuningRpm();
             * double range = mSuperstructure.getCurrentRange();
             * System.out.println("Tuning range: " + range + " = " + rpm);
             * mTuningFlywheelMap.put(new InterpolatingDouble(range), new
             * InterpolatingDouble(rpm)); mSuperstructure.incrementTuningRpm();
             * } }
             *
             * // Exhaust has highest priority for intake. if (wantsExhaust) {
             * mSuperstructure.setWantIntakeReversed(); } else if
             * (mControlBoard.getIntakeButton()) {
             * mSuperstructure.setWantIntakeOn(); } else if
             * (!mSuperstructure.isShooting()) {
             * mSuperstructure.setWantIntakeStopped(); }
             *
             * // Hanging has highest priority for feeder, followed by
             * exhausting, unjamming, and finally // feeding. if
             * (mControlBoard.getHangButton()) {
             * mSuperstructure.setWantedState(Superstructure.WantedState.HANG);
             * } else if (wantsExhaust) {
             * mSuperstructure.setWantedState(Superstructure.WantedState.EXHAUST
             * ); } else if (mControlBoard.getUnjamButton()) {
             * mSuperstructure.setWantedState(Superstructure.WantedState.UNJAM);
             * } else if (mControlBoard.getFeedButton()) {
             * mSuperstructure.setWantedState(Superstructure.WantedState.
             * MANUAL_FEED); } else if (mControlBoard.getRangeFinderButton()) {
             * mSuperstructure.setWantedState(Superstructure.WantedState.
             * RANGE_FINDING); } else {
             * mSuperstructure.setWantedState(Superstructure.WantedState.IDLE);
             * }
             *
             * if (mControlBoard.getFlywheelSwitch()) {
             * mSuperstructure.setClosedLoopRpm(Constants.kDefaultShootingRPM);
             * } else if (mControlBoard.getShooterOpenLoopButton()) {
             * mSuperstructure.setShooterOpenLoop(0); } else if
             * (mControlBoard.getShooterClosedLoopButton()) {
             * mSuperstructure.setClosedLoopRpm(Constants.kDefaultShootingRPM);
             * } else if (!mControlBoard.getHangButton() &&
             * !mControlBoard.getRangeFinderButton() &&
             * !mSuperstructure.isShooting()) {
             * mSuperstructure.setShooterOpenLoop(0); }
             *
             * if (mControlBoard.getBlinkLEDButton()) {
             * mLED.setWantedState(LED.WantedState.BLINK); }
             *
             * }
             */
            // boolean grab_gear = mControlBoard.getGrabGearButton();
            /*
             * ~!@ if (score_gear && grab_gear) {
             * mGearGrabber.setWantedState(WantedState.CLEAR_BALLS); } else if
             * (score_gear) { mGearGrabber.setWantedState(WantedState.SCORE); }
             * else if (grab_gear) {
             * mGearGrabber.setWantedState(WantedState.ACQUIRE); } else {
             * mGearGrabber.setWantedState(WantedState.IDLE); } DELETE THE BELOW
             * IF
             */

            // ~!@mSuperstructure.setActuateHopper(mControlBoard.getActuateHopperButton());
            allPeriodic();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();

            if (mAutoModeExecuter != null) {
                mAutoModeExecuter.stop();
            }
            mAutoModeExecuter = null;

            mEnabledLooper.stop();

            // Call stop on all our Subsystems.
            mSubsystemManager.stop();

            mDrive.setOpenLoop(DriveSignal.NEUTRAL);

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        /*
         * final double kVoltageThreshold = 0.15; if
         * (mCheckLightButton.getAverageVoltage() < kVoltageThreshold) {
         * mLED.setLEDOn(); } else { mLED.setLEDOff(); }
         */
        zeroAllSensors();
        allPeriodic();
        if ( DriverStation.getInstance().isFMSAttached() ) {
            if ( Constants.kGameSpecificMessage.length() != 3) {
                Constants.kGameSpecificMessage = DriverStation.getInstance().getGameSpecificMessage();
                Timer.delay(0.005);
            }
        } else {
            if ( Constants.kGameSpecificMessage.length() != 4) {
                Constants.kGameSpecificMessage = Constants.kMyGameMessages[ThreadLocalRandom.current().nextInt(0,8)];
            }
        }
        if (DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue) {
            Constants.kAllianceColor = "Blue";
        } else {
            Constants.kAllianceColor = "Red";
        }
        mRevDigitBoard.display(Constants.kGameSpecificMessage);

    }

    @Override
    public void testInit() {
        Timer.delay(0.5);

        boolean results = true;
        results &= Drive.getInstance().checkSystem();

        if (!results) {
            System.out.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
        } else {
            System.out.println("ALL SYSTEMS PASSED");
        }
    }

    @Override
    public void testPeriodic() {
        allPeriodic();
    }

    /**
     * Helper function that is called in all periodic functions
     */
    public void allPeriodic() {
        mSubsystemManager.outputToSmartDashboard();
        mSubsystemManager.writeToLog();
        mEnabledLooper.outputToSmartDashboard();
        // SmartDashboard.putBoolean("camera_connected",
        // mVisionServer.isConnected());


        ConnectionMonitor.getInstance().setLastPacketTime(Timer.getFPGATimestamp());
    }
}
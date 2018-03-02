package com.team6133.frc2018;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team6133.frc2018.auto.AutoModeExecuter;
import com.team6133.frc2018.loops.Looper;
import com.team6133.frc2018.subsystems.*;
import com.team6133.frc2018.vision.PiConnection;
import com.team6133.lib.util.CrashTracker;
import com.team6133.lib.util.DriveHelper;
import com.team6133.lib.util.DriveSignal;
import com.team6133.lib.util.drivers.RevDigitBoard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
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
            Launcher.getInstance(), Climber.getInstance(),
            ConnectionMonitor.getInstance() , LED.getInstance()
    ));
    // Get subsystem instances
    private Drive mDrive = Drive.getInstance();
    private Intake mIntake = Intake.getInstance();
    private Launcher mLauncher = Launcher.getInstance();
    private Climber mClimber = Climber.getInstance();

    private LED mLED = LED.getInstance();
    private AutoModeExecuter mAutoModeExecuter = null;
    // Initialize other helper objects
    private DriveHelper mDriveHelper = new DriveHelper();
    private ControlBoardInterface mControlBoard = ControlBoard.getInstance();

    private Looper mEnabledLooper = new Looper();

    private RevDigitBoard mRevDigitBoard = new RevDigitBoard();

    // ~!@private VisionServer mVisionServer = VisionServer.getInstance();

    // private AnalogInput mCheckLightButton = new
    // AnalogInput(Constants.kLEDOnId);

    public Robot() {
        CrashTracker.logRobotConstruction();
    }

    private void zeroAllSensors() {
        mSubsystemManager.zeroSensors();
        mDrive.zeroSensors();
    }

    private enum RobotDriveState {
        OPEN_LOOP,
        HEADING_SETPOINT,
        WANTS_AIM,
    }

    private enum RobotCubeState {
        INTAKE_FLOOR,
        INTAKE_STACK,
        EXHAUST_EXCHANGE,
        EXHAUST_SWITCH,
        LOAD_SHOOTER,
        HOLDING,
    }

    private RobotDriveState mDriveState;
    private RobotCubeState mCubeState;
    private boolean _intakeFloor         = false;
    private boolean _intakeStack         = false;
    private boolean _wantsAim            = false;
    private boolean _wantsLaunch         = false;
    private boolean _exhaustExchange     = false;
    private boolean _exhaustSwitch       = false;
    private boolean _loadLauncher        = false;
    private boolean _wantsCube           = false;
    private boolean _climb               = false;

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
            (new Thread(new PiConnection())).start();

            AutoModeSelector.initAutoModeSelector();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
        zeroAllSensors();
        mCubeState = RobotCubeState.HOLDING;
        mMatchState = MatchState.PRE_MATCH;
        mLED.setWantedState(LED.WantedState.PRE_MATCH);
    }

    /**
     * Initializes the robot for the beginning of autonomous mode (set
     * drivebase, intake and superstructure to correct states). Then gets the
     * correct auto mode from the AutoModeSelector
     *
     * @see AutoModeSelector
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
        mMatchState = MatchState.MID;
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
            mDriveState = RobotDriveState.OPEN_LOOP;
            // Start loopers
            mEnabledLooper.start();
            mDrive.setOpenLoop();

            zeroAllSensors();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
        mMatchState = MatchState.END;
    }

    /**
     * This function is called periodically during operator control.
     *
     * The code uses state machines to ensure that no matter what buttons the
     * driver presses, the robot behaves in a safe and consistent manner.
     *
     * Based on driver input, the code sets a desired state for each subsystem.
     * Each subsystem will constantly compare its desired and actual states and
     * act to bring the two closer.
     */
    @Override
    public void teleopPeriodic() {
        try {
            double timestamp = Timer.getFPGATimestamp();
            // Get Buttons
            boolean rotateLeftButton    = mControlBoard.getRotateLeftButton();
            boolean rotateRightButton   = mControlBoard.getRotateRightButton();
            boolean wants_aim_button    = mControlBoard.getWantsAlignButton();
            boolean wantsLaunchButton   = mControlBoard.getWantsLaunchButton();
            boolean intakeFloor         = mControlBoard.getIntakeFloorButton();
            boolean intakeStack         = mControlBoard.getIntakeStackButton();
            boolean exhaustExchange     = mControlBoard.getExhaustExchangeButton();
            boolean exhaustSwitch       = mControlBoard.getExhaustSwitchButton();
            boolean loadLauncher        = mControlBoard.getLoadLauncherButton();
            boolean extendClimber       = mControlBoard.getExtendClimbButton();
            boolean retractClimber      = mControlBoard.getRetractClimbButton();
            boolean climb               = mControlBoard.getClimbButton();
            boolean wantsCube           = mControlBoard.getWantsCubeIntakeButton();
            boolean wantsExhaust        = false;

            // Get the joystick signals and send the modified DriveSignal to the Drive class.
            // This MUST be done before calling any of the other Drive methods.
            mDrive.updateDriveSignal(mDriveHelper.mecDrive(mControlBoard.getThrottleX(),
                    mControlBoard.getThrottleY(), mControlBoard.getTwist()));

            // Prioritize the button input and set/update the robot accordingly.
            // Presently, these actions only happen *while* the button is pressed.
            // When no drive-related button is pressed, the robot will default to OPEN_LOOP
            if (wants_aim_button) {
                if (mDriveState != RobotDriveState.WANTS_AIM) {
                    mDrive.setAlignLaunch(timestamp);
                    mDriveState = RobotDriveState.WANTS_AIM;
                    Timer.delay(.005);
                }
                mDrive.updateAlignLaunch();
                // @TODO: Add more launcher functionality.
            } else if (rotateRightButton) {
                if (mDriveState != RobotDriveState.HEADING_SETPOINT || mDrive.getTargetHeading() != -90) {
                    mDrive.setTeleopHeadingSetpoint(-90, timestamp);
                    mDriveState = RobotDriveState.HEADING_SETPOINT;
                    Timer.delay(.005);
                }
                mDrive.updateTeleopHeadingSetpoint();
            } else if (rotateLeftButton) {
                if (mDriveState != RobotDriveState.HEADING_SETPOINT || mDrive.getTargetHeading() != 90) {
                    mDrive.setTeleopHeadingSetpoint(90, timestamp);
                    mDriveState = RobotDriveState.HEADING_SETPOINT;
                    Timer.delay(.005);
                }
                mDrive.updateTeleopHeadingSetpoint();
            } else {
                // No drive-related buttons have been pushed, so the default action is to drive OPEN_LOOP.
                if (!wantsCube) {
                    mDrive.setOpenLoop();
                    mDriveState = RobotDriveState.OPEN_LOOP;
                }
            }

            if (!wantsLaunchButton && _wantsLaunch) {
                // We just released the aim button. Time to fire!
                mLauncher.setWantsLaunch();
            } else if (wantsLaunchButton && !_wantsLaunch) {
                mLauncher.setWantedState(Launcher.WantedState.ALIGN);
            }

            if (!exhaustSwitch && _exhaustSwitch) {
                mLauncher.setWantsLaunch();
            } else if (exhaustSwitch && !_exhaustSwitch) {
                mLauncher.setWantedState(Launcher.WantedState.ALIGN_SWITCH);
            }

            if (intakeFloor) {
                /*if (mCubeState != RobotCubeState.INTAKE_FLOOR) {
                    mCubeState = RobotCubeState.INTAKE_FLOOR;
                    if (mIntake.hasCube())
                       mIntake.overrideHasCube();
                } else {
                    mCubeState = RobotCubeState.HOLDING;
                }
                if (mIntake.hasCube())
                    mIntake.overrideHasCube();*/
                mCubeState = RobotCubeState.INTAKE_FLOOR;
            } else if (_intakeFloor) {
                mIntake.setWantedState(Intake.WantedState.INTAKE_DONE);
                mCubeState = RobotCubeState.HOLDING;
            } else if (intakeStack && !_intakeStack) {
                if (mCubeState != RobotCubeState.INTAKE_STACK) {
                    mCubeState = RobotCubeState.INTAKE_STACK;
                } else {
                    mCubeState = RobotCubeState.HOLDING;
                }
                if (mIntake.hasCube())
                    mIntake.overrideHasCube();
            } else if (exhaustExchange && !_exhaustExchange) {
                mCubeState = RobotCubeState.EXHAUST_EXCHANGE;
                mIntake.setWantedState(Intake.WantedState.SCORE_EXCHANGE);
            } else if (!exhaustExchange && _exhaustExchange) {
                mIntake.setWantsExhaust();
                wantsExhaust = true;
            } else if (loadLauncher && !_loadLauncher) {
                mCubeState = RobotCubeState.LOAD_SHOOTER;
                mIntake.setWantedState(Intake.WantedState.LOAD_SHOOTER);
            }
            /*else {
                mIntake.setWantedState(Intake.WantedState.HOLDING);
                mCubeState = RobotCubeState.HOLDING;
            }*/

            // Set the wanted state of the Climber
            if (!climb && _climb) {
                mClimber.setWantedState(Climber.WantedState.IDLE);
                mCubeState = RobotCubeState.HOLDING;
            } else if (climb) {
                mClimber.setWantedState(Climber.WantedState.CLIMB);
                mCubeState = RobotCubeState.INTAKE_FLOOR;
            } else if (retractClimber) {
                mClimber.setWantedState(Climber.WantedState.RETRACT);
                mCubeState = RobotCubeState.INTAKE_FLOOR;
            } else if (extendClimber) {
                mClimber.setWantedState(Climber.WantedState.EXTEND);
                mCubeState = RobotCubeState.INTAKE_FLOOR;
            } else {
                mClimber.setWantedState(Climber.WantedState.IDLE);
            }


            switch (mCubeState) {
                case INTAKE_FLOOR:
                    mIntake.setWantedState(Intake.WantedState.ACQUIRE_FLOOR);
                    break;
                case INTAKE_STACK:
                    if (mIntake.hasCube()) {
                        mIntake.setWantedState(Intake.WantedState.HOLDING);
                        mCubeState = RobotCubeState.HOLDING;
                    } else {
                        mIntake.setWantedState(Intake.WantedState.ACQUIRE_STACK);
                    }
                    break;
                case EXHAUST_SWITCH:

                    if (mIntake.getWantedState() != Intake.WantedState.SCORE_SWITCH) {
                        mCubeState = RobotCubeState.HOLDING;
                    }
                    break;
                case EXHAUST_EXCHANGE:

                    if (mIntake.getWantedState() != Intake.WantedState.SCORE_EXCHANGE) {
                        mCubeState = RobotCubeState.HOLDING;
                    }
                    break;
                case LOAD_SHOOTER:


                    break;
                default:
                    mIntake.setWantedState(Intake.WantedState.HOLDING);
                    break;
            }

            allPeriodic();
            _exhaustExchange = exhaustExchange;
            _exhaustSwitch   = exhaustSwitch;
            _intakeFloor     = intakeFloor;
            _intakeStack     = intakeStack;
            _loadLauncher    = loadLauncher;
            _wantsAim        = wants_aim_button;
            _wantsLaunch     = wantsLaunchButton;
            _wantsCube       = wantsCube;
            _climb           = climb;
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    private enum MatchState {
        PRE_MATCH, MID, END
    }

    private MatchState mMatchState;

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
            mDrive.updateDriveSignal(DriveSignal.NEUTRAL);
            mDrive.setOpenLoop();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }

    }

    @Override
    public void disabledPeriodic() {
        if (mMatchState == MatchState.PRE_MATCH) {
            zeroAllSensors();
            // If we are connected to the FMS, then let's check every 5ms for the message.
            if ( DriverStation.getInstance().isFMSAttached() ) {
                if ( Constants.kGameSpecificMessage.length() != 3) {
                    Constants.kGameSpecificMessage = DriverStation.getInstance().getGameSpecificMessage();
                    Timer.delay(0.005);
                }
                Constants.kAllianceColor = DriverStation.getInstance().getAlliance();
                if (Constants.kAllianceColor == DriverStation.Alliance.Blue) {
                    mLED.setWantedState(LED.WantedState.ALLIANCE_BLUE);
                } else if (Constants.kAllianceColor == DriverStation.Alliance.Red) {
                    mLED.setWantedState(LED.WantedState.ALLIANCE_RED);
                }
            } else {    // We are not connected to the FMS, so randomly generate the message.
                if ( Constants.kGameSpecificMessage.length() != 4) {
                    Constants.kGameSpecificMessage = Constants.kMyGameMessages[ThreadLocalRandom.current().nextInt(0,8)];
                }
                // Update the constant for knowing our Alliance Color (used mostly for Auton)
                if (Constants.kAllianceColor == DriverStation.Alliance.Invalid)
                {
                    Constants.kAllianceColor = DriverStation.getInstance().getAlliance();
                }
            }
        } else if (mMatchState == MatchState.MID) {

        } else if (mMatchState == MatchState.END) {
            PiConnection.closeConnection();
            mLED.setWantedState(LED.WantedState.PRE_MATCH);
        }

        allPeriodic();


        // Display the game specific message on the REV Digit Board (connected to the MXP).
        mRevDigitBoard.display(Constants.kGameSpecificMessage);

    }
    @Override
    public void testInit() {
        Timer.delay(0.5);

        boolean results = true;
        //results &= Drive.getInstance().checkSystem();
        //results &= !Intake.getInstance().checkSystem();

        if (!results) {
            System.out.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
        } else {
            System.out.println("ALL SYSTEMS PASSED");
        }
        mDrive.setVoltageRightIRPD(216);
        mDrive.setVoltageLeftIRPD(100);
    }
    private int loops = 0;
    @Override
    public void testPeriodic() {
        allPeriodic();
        mDrive.mFrontSensor.update();
        if (++loops >= 200) {
            System.out.println("Limit switch Value:\t" + mIntake.checkSystem());
            System.out.println("Ultrasonic Value:\t" + mDrive.mFrontSensor.getAverageDistance());
            System.out.println("Left IRPD Sees Wall: "+mDrive.mLeftSensor.seesWall() + "\t||\tRight IRPD Sees Wall: " + mDrive.mRightSensor.seesWall());
            loops = 0;
        }
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
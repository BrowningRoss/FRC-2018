package com.team6133.frc2018;

import edu.wpi.first.wpilibj.Solenoid;

import com.team6133.lib.util.ConstantsBase;
import com.team6133.lib.util.InterpolatingDouble;
import com.team6133.lib.util.InterpolatingTreeMap;
import com.team6133.lib.util.math.PolynomialRegression;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;
import java.util.HashMap;

/**
 * A list of constants used by the rest of the robot code. This include physics
 * constants as well as constants determined through calibrations.
 */
public class Constants extends ConstantsBase {
	public static double kLooperDt = 0.005;



	/* ROBOT PHYSICAL CONSTANTS */

	// Wheels
	public static double kDriveWheelDiameterInches = 8.0;

	public static double kTrackWidthInches ;
	public static double kTrackScrubFactor ;

	// Geometry
	public static double kCenterToFrontBumperDistance ;
	public static double kCenterToIntakeDistance ;
	public static double kCenterToRearBumperDistance ;
	public static double kCenterToSideBumperDistance ;

	/* CONTROL LOOP GAINS */

	// PID gains for drive velocity loop
	// Units: setpoint, error, and output are in inches per second.
	public static double kDriveHighGearVelocityKp ;
	public static double kDriveHighGearVelocityKi ;
	public static double kDriveHighGearVelocityKd ;
	public static double kDriveHighGearVelocityKf ;
	public static int kDriveHighGearVelocityIZone ;
	public static double kDriveHighGearVelocityRampRate ;
	public static double kDriveHighGearNominalOutput ;
	public static double kDriveHighGearMaxSetpoint ;

	public static double kDriveVoltageCompensationRampRate = 0.0;

	// Turn to heading gains
	public static double kDriveTurnKp ;
	public static double kDriveTurnKi ;
	public static double kDriveTurnKv ;
	public static double kDriveTurnKffv ;
	public static double kDriveTurnKffa ;
	public static double kDriveTurnMaxVel ;
	public static double kDriveTurnMaxAcc ;


	// Do not change anything after this line unless you rewire the robot and
	// update the spreadsheet!
	// Port assignments should match up with the spreadsheet here:
	// https://docs.google.com/spreadsheets/d/12_Mrd6xKmxCjKtsWNpWZDqT7ukrB9-1KKFCuRrO4aPM/edit#gid=0

	/* TALONS */
	// (Note that if multiple talons are dedicated to a mechanism, any sensors
	// are attached to the master)

	// Drive
	public static final int kFrontLeftDriveId 	= 1;
	public static final int kRearLeftDriveId 	= 2;
	public static final int kFrontRightDriveId 	= 3;
	public static final int kRearRightDriveId 	= 4;


	// Solenoids
	// ~!@public static final int kShifterSolenoidId = 0; // PCM 0, Solenoid 0
	// ~!@public static final int kIntakeDeploySolenoidId = 1; // PCM 0,
	// Solenoid 1
	// ~!@public static final int kHopperSolenoidId = 2; // PCM 0, Solenoid 2
	// ~!@public static final int kGearWristSolenoid = 7; // PCM 0, Solenoid 7

	// Analog Inputs
	public static int kLEDOnId = 2;

	// Digital Outputs
	public static int kGreenLEDId = 9;
	public static int kRangeLEDId = 8;

	// Path following constants
	public static double kMinLookAhead 			= 12.0; // inches
	public static double kMinLookAheadSpeed 	= 9.0; // inches per second
	public static double kMaxLookAhead 			= 24.0; // inches
	public static double kMaxLookAheadSpeed 	= 75.0; // inches per second
	public static double kDeltaLookAhead 		= kMaxLookAhead - kMinLookAhead;
	public static double kDeltaLookAheadSpeed 	= kMaxLookAheadSpeed - kMinLookAheadSpeed;

	public static double kInertiaSteeringGain 	= 0.0; // angular velocity command
														// is multiplied by this
														// gain *
														// our speed
														// in inches per sec
	public static double kSegmentCompletionTolerance = 0.1; // inches
	public static double kPathFollowingMaxAccel ; // inches per second^2
	public static double kPathFollowingMaxVel ; // inches per second
	public static double kPathFollowingProfileKp ;
	public static double kPathFollowingProfileKi ;
	public static double kPathFollowingProfileKv ;
	public static double kPathFollowingProfileKffv ;
	public static double kPathFollowingProfileKffa ;
	public static double kPathStopSteeringDistance ;


	// Pose of the camera frame w.r.t. the robot frame
	public static double kCameraXOffset ;
	public static double kCameraYOffset ;
	public static double kCameraZOffset ;
	public static double kCameraPitchAngleDegrees ;
	public static double kCameraYawAngleDegrees ;
	public static double kCameraDeadband ;

	/**
	 * Make an {@link Solenoid} instance for the single-number ID of the
	 * solenoid
	 * 
	 * @param solenoidId
	 *            One of the kXyzSolenoidId constants
	 */
	public static Solenoid makeSolenoidForId(int solenoidId) {
		return new Solenoid(solenoidId / 8, solenoidId % 8);
	}

	@Override
	public String getFileLocation() {
		return "~/constants.txt";
	}

	/**
	 * @return the MAC address of the robot
	 */
	public static String getMACAddress() {
		try {
			Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
			StringBuilder ret = new StringBuilder();
			while (nwInterface.hasMoreElements()) {
				NetworkInterface nis = nwInterface.nextElement();
				if (nis != null) {
					byte[] mac = nis.getHardwareAddress();
					if (mac != null) {
						for (int i = 0; i < mac.length; i++) {
							ret.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
						}
						return ret.toString();
					} else {
						System.out.println("Address doesn't exist or is not accessible");
					}
				} else {
					System.out.println("Network Interface for the specified address is not found.");
				}
			}
		} catch (SocketException e) {
			e.printStackTrace();
		} catch (NullPointerException e) {
			e.printStackTrace();
		}
		return "";
	}
}
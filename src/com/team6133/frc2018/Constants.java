package com.team6133.frc2018;

import com.team6133.lib.util.ConstantsBase;
import edu.wpi.first.wpilibj.Solenoid;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

/**
 * A list of constants used by the rest of the robot code. This include physics
 * constants as well as constants determined through calibrations.
 */
public class Constants extends ConstantsBase {
    //Game Match Data
    public static String kGameSpecificMessage = "";
    public static String kAllianceColor = "Red";
    public static final String[] kMyGameMessages = new String[8];
    static {
        kMyGameMessages[0] = "LLLX";
        kMyGameMessages[1] = "LLRX";
        kMyGameMessages[2] = "LRLX";
        kMyGameMessages[3] = "LRRX";
        kMyGameMessages[4] = "RLLX";
        kMyGameMessages[5] = "RLRX";
        kMyGameMessages[6] = "RRLX";
        kMyGameMessages[7] = "RRRX";
    }


    // Drive
    public static final int kFrontLeftDriveId   = 7;
    public static final int kRearLeftDriveId    = 6;
    public static final int kFrontRightDriveId  = 4;
    public static final int kRearRightDriveId   = 1;

    // Intake
    public static final int kIntakeArmId        = 5;
    public static final int kIntakeLeftWheelId  = 2;
    public static final int kIntakeRightWheelId = 3;


    /* ROBOT PHYSICAL CONSTANTS */

    public static double kLooperDt = 0.005;
    // Wheels
    public static double kDriveWheelDiameterInches = 8.0;
    public static double kTrackWidthInches;
    public static double kTrackScrubFactor;

    /* CONTROL LOOP GAINS */
    // Geometry
    public static double kCenterToFrontBumperDistance;
    public static double kCenterToIntakeDistance;
    public static double kCenterToRearBumperDistance;
    public static double kCenterToSideBumperDistance;
    // PID gains for drive velocity loop
    // Units: setpoint, error, and output are in inches per second.
    // Turn to heading gains
    public static double kDriveTurnKp;
    public static double kDriveTurnKi;
    public static double kDriveTurnKv;


    // Do not change anything after this line unless you rewire the robot and
    // update the spreadsheet!
    // Port assignments should match up with the spreadsheet here:
    // https://docs.google.com/spreadsheets/d/12_Mrd6xKmxCjKtsWNpWZDqT7ukrB9-1KKFCuRrO4aPM/edit#gid=0

    /* TALONS */
    // (Note that if multiple talons are dedicated to a mechanism, any sensors
    // are attached to the master)
    public static double kDriveTurnKffv;
    public static double kDriveTurnKffa;
    public static double kDriveTurnMaxVel;
    public static double kDriveTurnMaxAcc;


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


    // Pose of the camera frame w.r.t. the robot frame
    public static double kCameraXOffset;
    public static double kCameraYOffset;
    public static double kCameraZOffset;
    public static double kCameraPitchAngleDegrees;
    public static double kCameraYawAngleDegrees;
    public static double kCameraDeadband;

    // Raspberry Pi Information
    public static String kVisionIP = "10.61.33.6";
    public static String kDriveStationIP = "10.61.33.5";
    public static String kRoborioIP = "10.61.33.2";
    public static String kRadioIP = "10.61.33.1";
    public static String kRoborioSubnetMask = "255.255.255.0";
    public static String kDriverStationSubnetMask = "255.0.0.0";
    public static String kVisionSubnetMask = "255.255.255.0";
    public static int    kVisionPort = 5800;
    public static int    kStreamingPort = 554;      // UDP/TCP 1180 - 1190: Camera Data
                                                    // TCP 1735: SmartDashboard
                                                    // UDP 1130: DS-to-Robot control data
                                                    // UDP 1140: Robot-to-DS status data
                                                    // HTTP 80: Camera/web interface
                                                    // HTTP 443: Camera/web interface (secure)
                                                    // UDP/TCP 554: Real-Time Streaming Protocol for h.264 camera streaming
                                                    // UDP/TCP 5800-5810: Team Use


    /**
     * Make an {@link Solenoid} instance for the single-number ID of the
     * solenoid
     *
     * @param solenoidId One of the kXyzSolenoidId constants
     */
    public static Solenoid makeSolenoidForId(int solenoidId) {
        return new Solenoid(solenoidId / 8, solenoidId % 8);
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

    @Override
    public String getFileLocation() {
        return "~/constants.txt";
    }
}
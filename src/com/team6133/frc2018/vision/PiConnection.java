package com.team6133.frc2018.vision;

import com.jcabi.ssh.SshByPassword;
import com.team6133.frc2018.Constants;
import edu.wpi.first.wpilibj.Timer;

import java.io.BufferedReader;
import java.io.DataOutputStream;
import java.io.InputStreamReader;
import java.net.InetAddress;
import java.net.Socket;
import com.jcabi.ssh.Shell;

public class PiConnection implements Runnable {
    private static PiConnection mInstance = null;
    public static PiConnection getInstance() {
        if (mInstance == null) {
            mInstance = new PiConnection();
        }
        return mInstance;
    }

    private static Socket mainSocket = null;
    private static Shell mShell;
    private static double serverStartTime;

    private static final String STATIC_IP = Constants.kVisionIP;

    public void run() {
        openSocket();
        initiateCamera();
    }

    public static boolean isConnected() {
        return (mainSocket != null || mShell != null);
    }

    public static boolean closeConnection() {
        if (isConnected()) {
            System.out.println("Closing connection to the Raspberry Pi");
            try {
                //mainSocket.close();
                //mainSocket = null;
                String stdout = new Shell.Plain(mShell).exec("sudo shutdown -P now");
                return true;
            } catch (Exception e) {
                System.out.println("ERROR closing socket:\t" + e.getMessage());
                return false;
            }
        } else {
        System.out.println("Warning: not connected. Cannot close connection.");
        return false;
        }
    }

    private static void openSocket() {
        waitUntilConnected();

        boolean reachable;
        try {
            reachable = InetAddress.getByName(STATIC_IP).isReachable(10);
        } catch (Exception e) {
            reachable = false;
        }

        if (reachable) {
            System.out.println("Raspberry Pi found at " + STATIC_IP);
            try {
                mShell = new SshByPassword("10.61.33.6", 5800, "pi", "r0botics");
            } catch (Exception e) {
                System.out.println("SSH ERROR: "+e.getMessage());
            }
            //mainSocket = connect(STATIC_IP);
        }
    }

    public static void waitUntilConnected() {
        boolean connected = false;
        System.out.println("Waiting until we are connected to gateway...");
        do {
            try {
                String ip = InetAddress.getLocalHost().getHostAddress();
                if (ip != null) {
                    connected = true;
                } else {
                    Thread.sleep(2000);
                }
            } catch (Exception e) {
            }
        } while (!connected);

        System.out.println("Connected to gateway.");
    }

    private static Socket connect(String ip) {
        boolean connected = false;
        do {
            try {
                System.out.println("Connecting to " + ip + ":5800");
                Socket clientSocket = new Socket(ip, 5800);
                DataOutputStream toServer = new DataOutputStream(clientSocket.getOutputStream());
                BufferedReader fromServer = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
                connected = true;

                String received;
                toServer.writeBytes("6133\n");
                received = fromServer.readLine();
                serverStartTime = Timer.getFPGATimestamp();
                System.out.println("RoboRIO received " + received + " back from the server.");
                if (received.equals("6133")) {
                    System.out.println("Connected to the Raspberry Pi!!!");
                    return clientSocket;
                }
            } catch (Exception e) {
                System.out.println("ERROR on connection:\t" + e.getMessage());
                try {
                    Thread.sleep(3000);
                } catch (Exception e2) {}
            }
        } while (!connected);
        return null;
    }

    private static void initiateCamera() {
        System.out.println("Starting camera server...");
        try {
            String stdout = new Shell.Plain(mShell).exec("webcam-streamer");
            System.out.println(stdout);
        } catch (Exception e) {
            System.out.println("Error executing command:\t" + e.getMessage());
        }
    }
}

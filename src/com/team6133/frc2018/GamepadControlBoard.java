package com.team6133.frc2018;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Contains the button mappings for the Gamepad control board. Like the drive
 * code, one instance of the GamepadControlBoard object is created upon startup,
 * then other methods request the singleton GamepadControlBoard instance.
 * Implements the ControlBoardInterface.
 *
 * @see ControlBoardInterface.java
 */
public class GamepadControlBoard implements ControlBoardInterface {

    private final Joystick mGamepad;

    protected GamepadControlBoard() {
        mGamepad = new Joystick(0);
    }

    @Override
    public double getThrottleX() {return 0;}

    @Override
    public double getThrottleY() {return 0;}

    @Override
    public double getTwist() {return 0;}

    @Override
    public boolean getRotateLeftButton() {
        return mGamepad.getRawButton(1);
    }

    @Override
    public boolean getRotateRightButton() {
        return mGamepad.getRawButton(4);
    }

    @Override
    public boolean getHangButton() {
        // A
        return mGamepad.getRawButton(1);
    }

    @Override
    public boolean getIntakeButton() {
        // L1
        return mGamepad.getRawButton(5);
    }

    @Override
    public boolean getFeedButton() {
        // X
        return false;
    }

    public boolean getGrabGearButton() {
        // L Trigger
        return mGamepad.getRawAxis(2) > 0.1;
    }

    public boolean getScoreGearButton() {
        return mGamepad.getRawAxis(3) > 0.1;
    }

    @Override
    public boolean getShooterOpenLoopButton() {
        // Y
        return mGamepad.getRawButton(4);
    }

    @Override
    public boolean getExhaustButton() {
        return false;
    }

    @Override
    public boolean getUnjamButton() {
        return false;
    }

    @Override
    public boolean getWantsLaunchButton() {
        // Back
        return false;
    }

    @Override
    public boolean getFlywheelSwitch() {
        return false;
    }

    public boolean getActuateHopperButton() {
        return mGamepad.getRawButton(9);
    }

    @Override
    public boolean getBlinkLEDButton() {
        return false;
    }

    @Override
    public boolean getRangeFinderButton() {
        // B
        return mGamepad.getRawButton(3);
    }

    @Override
    public boolean getWantGearDriveLimit() {
        return false;
    }

}
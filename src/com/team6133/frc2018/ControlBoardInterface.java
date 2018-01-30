package com.team6133.frc2018;

/**
 * A basic framework for robot controls that other controller classes implement
 */
public interface ControlBoardInterface {
    // DRIVER CONTROLS
    double getThrottleX();

    double getThrottleY();

    double getTwist();


    // OPERATOR CONTROLS
    boolean getRotateLeftButton();

    boolean getRotateRightButton();

    boolean getFeedButton();

    boolean getIntakeButton();

    boolean getShooterOpenLoopButton();

    boolean getExhaustButton();

    boolean getUnjamButton();

    boolean getWantsLaunchButton();

    boolean getFlywheelSwitch();

    boolean getHangButton();

    boolean getBlinkLEDButton();

    boolean getRangeFinderButton();

    boolean getWantGearDriveLimit();
}
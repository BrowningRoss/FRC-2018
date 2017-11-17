package com.team6133.frc2018.auto.actions;

import edu.wpi.first.wpilibj.Timer;

import com.team6133.frc2018.RobotState;
import com.team6133.frc2018.ShooterAimingParameters;
import com.team6133.frc2018.subsystems.Drive;
import com.team6133.frc2018.subsystems.LED;
import com.team6133.lib.util.DriveSignal;

import java.util.Optional;

/**
 * Runs the drivebase in open loop until the robot sees the boiler.
 *
 * @see Action
 */
public class OpenLoopUntilSeesTargetAction implements Action {

    RobotState mState = RobotState.getInstance();
    double left;
    double right;

    public OpenLoopUntilSeesTargetAction(double left, double right) {
        this.left = left;
        this.right = right;
    }

    public boolean isFinished() {
        double now = Timer.getFPGATimestamp();
        Optional<ShooterAimingParameters> aimParams = mState.getAimingParameters();
        if (aimParams.isPresent() && Math.abs(now - aimParams.get().getLastSeenTimestamp()) < 0.5) {
            return true;
        }
        return false;
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub

    }

    @Override
    public void done() {
        // TODO Auto-generated method stub

    }

    @Override
    public void start() {
        LED.getInstance().setWantedState(LED.WantedState.FIND_RANGE);
        Drive.getInstance().setOpenLoop(new DriveSignal(left, right));
    }

}
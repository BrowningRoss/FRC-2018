package com.team6133.frc2018.auto.actions;

import com.team6133.frc2018.Constants;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;

/**
 * Composite action, running all sub-actions at the same time All actions are
 * started then updated until all actions report being done.
 */
public class ParallelAction implements Action {

    private final ArrayList<Action> mActions;

    public ParallelAction(List<Action> actions) {
        mActions = new ArrayList<>(actions.size());
        for (Action action : actions) {
            mActions.add(action);
        }
    }

    @Override
    public boolean isFinished() {
        boolean all_finished = true;
        for (Action action : mActions) {
            if (!action.isFinished()) {
                all_finished = false;
            }
        }
        return all_finished || (Timer.getFPGATimestamp() - Constants.Robot_Auton_Start_Time) > 15;
    }

    @Override
    public void update() {
        for (Action action : mActions) {
            action.update();
        }
    }

    @Override
    public void done() {
        for (Action action : mActions) {
            action.done();
        }
    }

    @Override
    public void start() {
        for (Action action : mActions) {
            action.start();
        }
    }
}
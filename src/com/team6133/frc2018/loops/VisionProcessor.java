package com.team6133.frc2018.loops;

import com.team6133.frc2018.vision.VisionUpdate;
import com.team6133.frc2018.vision.VisionUpdateReceiver;


public class VisionProcessor implements Loop, VisionUpdateReceiver {
    static VisionProcessor instance_ = new VisionProcessor();
    VisionUpdate update_ = null;

    public static VisionProcessor getInstance() {
        return instance_;
    }

    VisionProcessor() {
    }

    @Override
    public void onStart(double timestamp) {
    }

    @Override
    public void onLoop(double timestamp) {
        VisionUpdate update;
        synchronized (this) {
            if (update_ == null) {
                return;
            }
            update = update_;
            update_ = null;
        }
        //---robot_state_.addVisionUpdate(update.getCapturedAtTimestamp(), update.getTargets());
    }

    @Override
    public void onStop(double timestamp) {
        // no-op
    }

    @Override
    public synchronized void gotUpdate(VisionUpdate update) {
        update_ = update;
    }

}
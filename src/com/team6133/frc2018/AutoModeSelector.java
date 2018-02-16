package com.team6133.frc2018;

import com.team6133.frc2018.auto.AutoModeBase;
import com.team6133.frc2018.auto.StartingPosition;
import com.team6133.frc2018.auto.modes.StandStillMode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.json.simple.JSONArray;

import java.util.function.Supplier;

/**
 * Class that allows a user to select which autonomous mode to execute from the
 * web dashboard.
 */
public class AutoModeSelector {

    public static final String AUTO_OPTIONS_DASHBOARD_KEY = "auto_options";
    public static final String SELECTED_AUTO_MODE_DASHBOARD_KEY = "selected_auto_mode";
    //public static final String STARTING_POSITION_DASHBOARD_KEY = "starting_position";
    //public static final String SELECTED_STARTING_POISITION_DASHBOARD_KEY = "selected_starting_position";
    private static final AutoModeCreator mDefaultMode = new AutoModeCreator("Stand Still",
            () -> new StandStillMode());
    private static final AutoModeCreator[] mAllModes = {
            new AutoModeCreator("Standstill", () -> new StandStillMode()),};
    /*private static final StartingPositionCreator[] mAllPositions = {
            new StartingPositionCreator("Left", StartingPosition.LEFT),
            new StartingPositionCreator("Center", StartingPosition.CENTER),
            new StartingPositionCreator("Right", StartingPosition.RIGHT)
    };
    private static final StartingPositionCreator mDefaultStartingPosition =
            new StartingPositionCreator("Left", StartingPosition.LEFT);
    */
    public static void initAutoModeSelector() {
        JSONArray modesArray = new JSONArray();
        for (AutoModeCreator mode : mAllModes) {
            modesArray.add(mode.mDashboardName);
        }
        /*JSONArray positionArray = new JSONArray();
        for (StartingPositionCreator s : mAllPositions) {
            positionArray.add(s.mDashboardName);
        }*/
        SmartDashboard.putString(AUTO_OPTIONS_DASHBOARD_KEY, modesArray.toString());
        SmartDashboard.putString(SELECTED_AUTO_MODE_DASHBOARD_KEY, mDefaultMode.mDashboardName);
        //SmartDashboard.putString(STARTING_POSITION_DASHBOARD_KEY, positionArray.toString());
        //SmartDashboard.putString(SELECTED_STARTING_POISITION_DASHBOARD_KEY, mDefaultStartingPosition.mDashboardName);
    }

    public static AutoModeBase getSelectedAutoMode() {
        String selectedModeName = SmartDashboard.getString(SELECTED_AUTO_MODE_DASHBOARD_KEY, "NO SELECTED MODE!!!!");
        for (AutoModeCreator mode : mAllModes) {
            if (mode.mDashboardName.equals(selectedModeName)) {
                return mode.mCreator.get();
            }
        }
        DriverStation.reportError("Failed to select auto mode: " + selectedModeName, false);
        return mDefaultMode.mCreator.get();
    }
    /*
    public static StartingPosition getSelectedStartingPosition() {
        String selectedPosition = SmartDashboard.getString(SELECTED_STARTING_POISITION_DASHBOARD_KEY, "NO SELECTED POSITION!!!");
        for (StartingPositionCreator position : mAllPositions) {
            if (position.mDashboardName.equals(selectedPosition)) {
                return position.mPosition;
            }
        }
        DriverStation.reportError("Failed to select starting position: " + selectedPosition, false);
        return mDefaultStartingPosition.mPosition;
    }
    */
    private static class AutoModeCreator {
        private final String mDashboardName;
        private final Supplier<AutoModeBase> mCreator;

        private AutoModeCreator(String dashboardName, Supplier<AutoModeBase> creator) {
            mDashboardName = dashboardName;
            mCreator = creator;
        }
    }
    /*
    private static class StartingPositionCreator {
        private final String mDashboardName;
        private final StartingPosition mPosition;

        private StartingPositionCreator(String dashboardName, StartingPosition position) {
            mDashboardName = dashboardName;
            mPosition = position;
        }
    }*/
}
package com.team6133.frc2018;

import com.team6133.frc2018.auto.AutoModeBase;
import com.team6133.frc2018.auto.modes.*;
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
    private static final AutoModeCreator mDefaultMode = new AutoModeCreator("Start Left | L Scale > L Switch > R Switch", () -> new LEFT_PreferScale_EitherSwitch());
    private static final AutoModeCreator[] mAllModes = {
            new AutoModeCreator("Start Left | L Scale > L Switch > R Switch", () -> new LEFT_PreferScale_EitherSwitch()),
            new AutoModeCreator("Start Left | L Scale > L Switch > R Scale", () -> new LEFT_LScale_LSwitch_RScale()),
            new AutoModeCreator("Start Left | L Scale > R Scale", () -> new LEFT_LScale_RScale()),
            new AutoModeCreator("Start Left | L Scale > L Switch > Cross", () -> new LEFT_PreferScale()),
            new AutoModeCreator("Start Left | L Switch > R Switch", () -> new LEFT_EitherSwitch_NoScale()),
            new AutoModeCreator("Start Left | L Switch > L Scale > R Switch", () -> new LEFT_PreferLSwitch_LScale_RSwitch()),
            new AutoModeCreator("Start Left | L Switch > L Scale > Cross", () -> new LEFT_PreferSwitch()),
            new AutoModeCreator("Start Left | L Switch > Cross", () -> new LEFT_SwitchNoScale()),
            new AutoModeCreator("Start Center | Score Switch NEAR Side", () -> new CENTER_ScoreSwitchNear()),
            new AutoModeCreator("Start Center | Score Switch FAR Side", () -> new CENTER_ScoreSwitchFar()),
            new AutoModeCreator("Start Right | R Scale > R Switch > Cross", () -> new RIGHT_PreferScale())
    };

    public static void initAutoModeSelector() {
        JSONArray modesArray = new JSONArray();
        for (AutoModeCreator mode : mAllModes) {
            modesArray.add(mode.mDashboardName);
        }

        SmartDashboard.putString(AUTO_OPTIONS_DASHBOARD_KEY, modesArray.toString());
        SmartDashboard.putString(SELECTED_AUTO_MODE_DASHBOARD_KEY, mDefaultMode.mDashboardName);
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

    private static class AutoModeCreator {
        private final String mDashboardName;
        private final Supplier<AutoModeBase> mCreator;

        private AutoModeCreator(String dashboardName, Supplier<AutoModeBase> creator) {
            mDashboardName = dashboardName;
            mCreator = creator;
        }
    }
}
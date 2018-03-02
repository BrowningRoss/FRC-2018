package com.team6133.frc2018.auto.modes;

import com.team6133.frc2018.Constants;
import com.team6133.frc2018.auto.AutoModeBase;
import com.team6133.frc2018.auto.AutoModeEndedException;
import edu.wpi.first.wpilibj.DriverStation;

public class LEFT_LScale_LSwitch_RScale extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        AutoModeBase determinedAutoMode = new Option_StartLeft_EndScoreScale();
        if (Constants.kGameSpecificMessage.length() != 3) {
            if (DriverStation.getInstance().isFMSAttached())
                Constants.kGameSpecificMessage = DriverStation.getInstance().getGameSpecificMessage();
            else
                Constants.kGameSpecificMessage = "LLLX";
        }
        if (Constants.kGameSpecificMessage.charAt(1) == 'R' && Constants.kGameSpecificMessage.charAt(0) == 'L') {
            determinedAutoMode = new Option_StartLeft_EndScoreLeftSwitch();
            System.out.println(Constants.kGameSpecificMessage + "\t-> Starting Score Left Switch");
        } else if (Constants.kGameSpecificMessage.charAt(1) == 'R') {
            determinedAutoMode = new Option_StartLeft_EndScoreRightScale();
            System.out.println(Constants.kGameSpecificMessage + "\t-> Starting Score Right Scale - FINGERS CROSSED!");
        }else {
            System.out.println(Constants.kGameSpecificMessage + "\t-> Starting Score Left Scale");
        }

        determinedAutoMode.run();
    }
}

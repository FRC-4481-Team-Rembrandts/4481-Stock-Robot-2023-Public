package frc.team4481.robot.auto.selector;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4481.lib.auto.AllianceColor;
import frc.team4481.lib.auto.mode.AutoModeBase;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.robot.auto.modes.*;


/**
 * Class for selecting of autonomous mode via the driver station.
 *
 * Inspired by Team 254
 */
public class AutoModeSelector {
    private final SubsystemHandler mSubsystemHandler = SubsystemHandler.getInstance();

    private AutoModeBase mAutoMode;

    private AutoModeList mSelectedAutoMode = null;
    private AllianceColor mSelectedAlliance = null;

    private final SendableChooser<AutoModeList> mModeChooser;
    private final SendableChooser<AllianceColor> mAllianceChooser;

    private final DoNothing mDoNothing = new DoNothing();

    /**
     * Creates an autonomous mode selector which is visible on the driver station.
     */
    public AutoModeSelector() {
        mAllianceChooser = new SendableChooser<>();
        mAllianceChooser.setDefaultOption("RED", AllianceColor.RED);
        mAllianceChooser.addOption("BLUE", AllianceColor.BLUE);


        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("DO NOTHING", AutoModeList.DO_NOTHING);

        SmartDashboard.putData(mAllianceChooser);
        SmartDashboard.putData(mModeChooser);
        SmartDashboard.putString("FMS Detected Alliance Color", DriverStation.getAlliance().name());
    }

    /**
     * Update the selected autonomous mode based in the driver station input
     */
    public void updateAutoModeSelector() {
        AutoModeList desiredMode = mModeChooser.getSelected();
        AllianceColor desiredAlliance = mAllianceChooser.getSelected();

        if (mSelectedAutoMode != desiredMode || mSelectedAlliance != desiredAlliance) {
            mAutoMode = getAutoModeForParams(desiredMode, desiredAlliance);

            mSelectedAutoMode = desiredMode;
            mSelectedAlliance = desiredAlliance;
        }

        if (mAutoMode != null) {
            SmartDashboard.putString("Currently Selected Autonomous Mode", mAutoMode.toString());
        } else {
            SmartDashboard.putString("Currently Selected Autonomous Mode", "NoneSelected");
        }

    }

    /**
     * Get the desired instance of the autonomous mode based on the alliance color and selected autonomous mode
     * from the driver station
     *
     * @param pAutoMode Autonomous mode enum
     * @param pAlliance Alliance color enum
     *
     * @return Instance of autonomous mode to run
     */
    private AutoModeBase getAutoModeForParams(AutoModeList pAutoMode, AllianceColor pAlliance) {
        if (pAutoMode == AutoModeList.DO_NOTHING)
        {
            return mDoNothing;
        }
        switch (pAlliance){
            case RED:
                switch(pAutoMode){
                    case DO_NOTHING:
                        return mDoNothing;
                }
                break;
            case BLUE:
                switch(pAutoMode){
                    case DO_NOTHING:
                        return mDoNothing;
                }
                break;
        }

        return null;
    }

    public AutoModeBase getAutoMode() {return mAutoMode;}

    public void reset() {
        mAutoMode = null;
        mSelectedAutoMode = null;
        mSelectedAlliance = null;
    }
}


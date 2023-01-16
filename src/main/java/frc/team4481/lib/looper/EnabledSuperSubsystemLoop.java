package frc.team4481.lib.looper;

import frc.team4481.lib.subsystems.SubsystemBase;
import frc.team4481.lib.subsystems.SubsystemHandler;

import java.util.List;

/**
 * Base loop for execution of the loop functions inside each subsystem when the robot is enabled.
 */
public class EnabledSuperSubsystemLoop implements Loop {
    private final List<SubsystemBase> mAllSubsystems;

    public EnabledSuperSubsystemLoop(SubsystemHandler mSubsystemManger){
        this.mAllSubsystems = mSubsystemManger.getSubsystems();
    }

    @Override
    public void onStart(double timestamp) {
        mAllSubsystems.forEach(s -> s.onStart(timestamp));
    }

    @Override
    public void onLoop(double timestamp) {
        mAllSubsystems.forEach(SubsystemBase::readPeriodicInputs);
        mAllSubsystems.forEach(s -> s.onLoop(timestamp));
        mAllSubsystems.forEach(SubsystemBase::writePeriodicOutputs);
        mAllSubsystems.forEach(SubsystemBase::outputData);
    }

    @Override
    public void onStop(double timestamp) {
        mAllSubsystems.forEach(s -> s.onStop(timestamp));
    }
}

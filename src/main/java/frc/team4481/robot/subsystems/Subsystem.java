package frc.team4481.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import frc.team4481.lib.subsystems.SubsystemBase;
import frc.team4481.lib.subsystems.SubsystemHandler;

public class Subsystem extends SubsystemBase<SubsystemManager> {
    private final SubsystemHandler mSubsystemHandler = SubsystemHandler.getInstance();


    public Subsystem(){
        name = "Subsystem";
        subsystemManager = new SubsystemManager();
    }

    @Override
    public void onStart(double timestamp) {
        if (DriverStation.isAutonomous()){
            subsystemManager.setControlState(SubsystemManager.controlState.DISABLED);
        }else{
            subsystemManager.setControlState(SubsystemManager.controlState.DISABLED);
        }

        zeroSensors();
    }

    @Override
    public void readPeriodicInputs() {

    }

    @Override
    public void onLoop(double timestamp) {
        switch (subsystemManager.getControlState()) {
            case DISABLED:

                break;
        }
    }
    @Override
    public void writePeriodicOutputs() {

    }

    @Override
    public void onStop(double timestamp) {
        terminate();
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void terminate() {

        subsystemManager.setControlState(SubsystemManager.controlState.DISABLED);

    }

    @Override
    public void outputData() {

    }

}

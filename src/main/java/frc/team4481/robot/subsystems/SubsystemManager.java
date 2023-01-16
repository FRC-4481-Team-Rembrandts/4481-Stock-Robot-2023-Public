package frc.team4481.robot.subsystems;

import frc.team4481.lib.subsystems.SubsystemManagerBase;

public class SubsystemManager extends SubsystemManagerBase {
    private controlState currentControlState = controlState.DISABLED;

    public enum controlState {
        DISABLED,
        ENABLED,
    }

    public void setControlState(controlState pControlState) {
        currentControlState = pControlState;
    }

    public controlState getControlState() {
        return currentControlState;
    }
}

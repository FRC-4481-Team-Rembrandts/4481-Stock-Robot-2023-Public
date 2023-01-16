package frc.team4481.robot.HIDlayout;

import frc.team4481.lib.hid.HIDLayout;
import frc.team4481.lib.controller.ControlDevice;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.lib.throwable.HardwareException;
import frc.team4481.robot.subsystems.Subsystem;
import frc.team4481.robot.subsystems.SubsystemManager;

public class Layout extends HIDLayout {
    private final SubsystemHandler mSubsystemHandler = SubsystemHandler.getInstance();

    // TODO add required subsystems
    private Subsystem subsystem;
    private SubsystemManager subsystemManager;


    public Layout(ControlDevice driver, ControlDevice operator) {
        super(driver, operator);
    }


    @Override
    public void getSubsystemManagers() {
        // TODO add required subsystems
        subsystem = (Subsystem) mSubsystemHandler.getSubsystemByClass(Subsystem.class);
        subsystemManager = subsystem.getSubsystemManager();
    }

    @Override
    public void updateOrange() throws HardwareException {

    }

    @Override
    public void updateBlack() throws HardwareException {

    }
}

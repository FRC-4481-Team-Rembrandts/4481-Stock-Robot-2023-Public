package frc.team4481.robot;

import frc.team4481.lib.hid.HIDLayout;
import frc.team4481.lib.controller.ControlDevice;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.lib.throwable.HardwareException;
import frc.team4481.robot.HIDlayout.Layout;

public class HIDManager {
    public static HIDManager mInstance = null;

    private HIDManager() {
    }

    ControlDevice mDriver = new ControlDevice(0);
    ControlDevice mOperator = new ControlDevice(1);

    HIDLayout layout = new Layout(mDriver, mOperator);

    public static HIDManager getInstance() {
        if (mInstance == null) {
            mInstance = new HIDManager();
        }
        return mInstance;
    }

    public void getControllers(SubsystemHandler subsystemHandler) {
        layout.getSubsystemManagers();
    }

    public void update() {
        try {
            layout.updateOrange();
            layout.updateBlack();
        } catch (HardwareException e) {
            e.printStackTrace();
        }
    }
}

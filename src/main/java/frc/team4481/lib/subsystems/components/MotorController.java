package frc.team4481.lib.subsystems.components;

@Deprecated
public enum MotorController {
    SPARKMAX,
    TALONSRX,
    FALCON,
    VICTORSPX;
    public enum ControlMode{
        PERCENTOUTPUT,
        CURRENT,
        PID,
        FOLLOWER,
        PIDF,
    }
    public enum EncoderValue{
        POSITION,
        VELOCITY,
        PERCENT
    }
}

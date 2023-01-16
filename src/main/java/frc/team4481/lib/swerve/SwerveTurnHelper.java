package frc.team4481.lib.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class SwerveTurnHelper
{
    public double offsetDegrees;
    public CANSparkMax turnMotor;
    private SparkMaxPIDController controller;
    public ArmFeedforward FFController;
    public DutyCycleEncoder absoluteEncoder;

    /**
     *
     * @param motorID
     * @param sensorID
     * @param offsetDegrees
     * @param PIDValues
     * @param FFValues
     */
    public SwerveTurnHelper(
            int motorID,
            int sensorID,
            double offsetDegrees,
            PIDValueContainer PIDValues,
            FFValueContainer FFValues
            )
    {
        this.offsetDegrees = offsetDegrees;

        turnMotor = new CANSparkMax(motorID, CANSparkMax.MotorType.kBrushless);
        turnMotor.restoreFactoryDefaults();
        turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        controller = turnMotor.getPIDController();
        controller.setPositionPIDWrappingEnabled(true);
        controller.setPositionPIDWrappingMinInput(-Math.PI);
        controller.setPositionPIDWrappingMaxInput(Math.PI);
        controller.setP(PIDValues.kP);
        controller.setI(PIDValues.kI);
        controller.setD(PIDValues.kD);
//        turnMotor.burnFlash();

        FFController = new ArmFeedforward(
                FFValues.kS,
                0.0,
                FFValues.kV,
                FFValues.kA
        );

        absoluteEncoder = new DutyCycleEncoder(sensorID);
    }
}

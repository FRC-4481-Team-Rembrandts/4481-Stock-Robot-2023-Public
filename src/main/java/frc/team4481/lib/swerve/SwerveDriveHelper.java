package frc.team4481.lib.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class SwerveDriveHelper 
{
    public CANSparkMax driveMotor;
    private SparkMaxPIDController controller;
    public SimpleMotorFeedforward FFController;

    /**
     *
     * @param motorID
     * @param isInverted
     * @param PIDValues
     * @param FFValues
     */
    public SwerveDriveHelper(
            int motorID,
            boolean isInverted,
            PIDValueContainer PIDValues,
            FFValueContainer FFValues
    )
    {
        driveMotor = new CANSparkMax(motorID, CANSparkMax.MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        driveMotor.setInverted(isInverted);

        controller = driveMotor.getPIDController();
        controller.setP(PIDValues.kP);
        controller.setI(PIDValues.kI);
        controller.setD(PIDValues.kD);
//        driveMotor.burnFlash();

        FFController = new SimpleMotorFeedforward(
                FFValues.kS,
                FFValues.kV,
                FFValues.kA
        );
    }
}

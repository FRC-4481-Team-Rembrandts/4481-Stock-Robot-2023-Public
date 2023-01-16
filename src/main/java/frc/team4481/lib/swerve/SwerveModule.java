package frc.team4481.lib.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule
{
    // Driving
    private CANSparkMax driveMotor;
    private RelativeEncoder driveEncoder;
    private SparkMaxPIDController drivePIDController;
    private SimpleMotorFeedforward driveFFController;

    // Turning
    private CANSparkMax turnMotor;
    private RelativeEncoder turnEncoder;
    private SparkMaxPIDController turnPIDController;
    private DutyCycleEncoder absoluteTurnEncoder;
    private ArmFeedforward turnFFController;
    private double absoluteTurnOffset; // Degrees
    private Rotation2d turnOffset;

    private SwerveModulePhysicalConstants consts;

    // Current limits
    private int stallLimitDrive;
    private int freeLimitDrive;
    private int stallLimitTurn;
    private int freeLimitTurn;

    public SwerveModule(
            SwerveTurnHelper turnHelper,
            SwerveDriveHelper driveHelper,
            SwerveModulePhysicalConstants swerveModulePhysicalConstants
    )
    {
        // Set turn stuff
        turnMotor = turnHelper.turnMotor;
        turnEncoder = turnMotor.getEncoder();
        turnPIDController = turnMotor.getPIDController();
        turnFFController = turnHelper.FFController;
        absoluteTurnEncoder = turnHelper.absoluteEncoder;

        // Set drive stuff
        driveMotor = driveHelper.driveMotor;
        driveEncoder = driveMotor.getEncoder();
        drivePIDController = driveMotor.getPIDController();
        driveFFController = driveHelper.FFController;

        // Set absolute encoder offset
        absoluteTurnOffset = turnHelper.offsetDegrees;

        // Physical constants
        consts = swerveModulePhysicalConstants;

        driveEncoder.setVelocityConversionFactor(consts.drive_velocity_conversion_factor);
        driveEncoder.setPositionConversionFactor(consts.drive_position_conversion_factor);
        turnEncoder.setVelocityConversionFactor(consts.turn_velocity_conversion_factor);
        turnEncoder.setPositionConversionFactor(consts.turn_position_conversion_factor);

//        driveMotor.burnFlash();
//        turnMotor.burnFlash();
        // Maybe add timeout
    }

    /**
     * Gets the current {@code SwerveModuleState} of this module
     *
     * @return current state of this module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getAbsoluteAngle());
    }

    /**
     * Sets the desired velocity and angle of this swerve module
     *
     * @param desiredState the desired {@code SwerveModuleState} of this module
     */
    public void setDesiredState(SwerveModuleState desiredState, double desiredTurnSpeed)
    {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001)
        {
            stopModule();
            return;
        }
        // Optimize the desired state such that the wheel has to turn the smallest possible angle
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getTurnAngle());

        // Turning
        double targetAngle = optimizedState.angle.getRadians();

        final double turnArbFFComponent = turnFFController.calculate(targetAngle, desiredTurnSpeed);
        turnPIDController.setReference(
                targetAngle,
                CANSparkMax.ControlType.kPosition,
                0,
                turnArbFFComponent,
                SparkMaxPIDController.ArbFFUnits.kVoltage
        );
        SmartDashboard.putNumber("Target angle " + turnMotor.getDeviceId(), targetAngle);


        // Driving
        double targetVelocity = optimizedState.speedMetersPerSecond;

        double arbFFComponent = driveFFController.calculate(targetVelocity);
        drivePIDController.setReference(
                targetVelocity,
                CANSparkMax.ControlType.kVelocity,
                0,
                arbFFComponent,
                SparkMaxPIDController.ArbFFUnits.kVoltage
        );
        SmartDashboard.putNumber("Target velocity " + driveMotor.getDeviceId(), targetVelocity);
    }

    /**
     * Gets the current turning angle of the swerve module
     *
     * @return current {@code Rotation2d} of turning angle
     */
    private Rotation2d getTurnAngle() {
        return new Rotation2d(turnEncoder.getPosition());
    }

    /**
     * Gets the current absolute angle of the swerve module
     *
     * @return current {@code Rotation2d} of absolute turning angle
     */
    public Rotation2d getAbsoluteAngle()
    {
        double rotationDegrees = 360 - absoluteTurnEncoder.getAbsolutePosition() * 360;
        double rotationDegreesOffset = rotationDegrees - absoluteTurnOffset;
        return Rotation2d.fromDegrees(MathUtil.inputModulus(rotationDegreesOffset,-180,180));
    }

    /**
     * Calibrates {@code turnOffset}
     */
    public void calibrateTurnEncoder() {
        turnOffset = getAbsoluteAngle();
        turnEncoder.setPosition(turnOffset.getRadians());
    }

    /**
     * Gets the current velocity of the swerve module in m/s.
     *
     * @return current velocity in m/s
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveEncoder.getPosition(),
                new Rotation2d(turnEncoder.getPosition()));
    }

    /**
     * Set both motors of o {@code SwerveModule} to 0 output.
     */
    private void stopModule()
    {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    // Methods to edit Swerve module parameters
    public void setInverted(boolean isInverted) {
        driveMotor.setInverted(isInverted);
    }

    public boolean getInverted() {
        return driveMotor.getInverted();
    }

    public void setIdleMode(CANSparkMax.IdleMode mode) {
        driveMotor.setIdleMode(mode);
    }

    public CANSparkMax.IdleMode getIdleMode() {
        return driveMotor.getIdleMode();
    }

    public void setClosedLoopRampRate(double rate) {
        driveMotor.setClosedLoopRampRate(rate);
    }

    public double getClosedLoopRampRate() {
        return driveMotor.getClosedLoopRampRate();
    }

    public void setDriveCurrentLimit(int stallLimitDrive, int freeLimitDrive) {
        this.stallLimitDrive = stallLimitDrive;
        this.freeLimitDrive = freeLimitDrive;
        driveMotor.setSmartCurrentLimit(stallLimitDrive, freeLimitDrive);
    }

    public void setTurnCurrentLimit(int stallLimitTurn, int freeLimitTurn) {
        this.stallLimitTurn = stallLimitTurn;
        this.freeLimitTurn = freeLimitTurn;
        turnMotor.setSmartCurrentLimit(stallLimitTurn, freeLimitTurn);
    }

    public int getStallLimitDrive() {
        return stallLimitDrive;
    }

    public int getFreeLimitDrive() {
        return freeLimitDrive;
    }

    public int getStallLimitTurn() {
        return stallLimitTurn;
    }

    public int getFreeLimitTurn() {
        return freeLimitTurn;
    }
}
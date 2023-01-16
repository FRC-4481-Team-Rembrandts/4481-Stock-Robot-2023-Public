package frc.team4481.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4481.lib.path.AdaptivePurePursuitController;
import frc.team4481.lib.subsystems.SubsystemBase;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.lib.swerve.*;

import static frc.team4481.robot.Constants.*;

public class Drivetrain extends SubsystemBase<DrivetrainManager> {
    private final SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();

    private final Translation2d frontLeftLocation = new Translation2d(DRIVETRAIN_WHEELBASE_DISTANCE, DRIVETRAIN_WHEELBASE_DISTANCE);
    private final Translation2d frontRightLocation = new Translation2d(DRIVETRAIN_WHEELBASE_DISTANCE, -DRIVETRAIN_WHEELBASE_DISTANCE);
    private final Translation2d backLeftLocation = new Translation2d(-DRIVETRAIN_WHEELBASE_DISTANCE, DRIVETRAIN_WHEELBASE_DISTANCE);
    private final Translation2d backRightLocation = new Translation2d(-DRIVETRAIN_WHEELBASE_DISTANCE, -DRIVETRAIN_WHEELBASE_DISTANCE);

    private SwerveModule frontLeft;
    private SwerveModule frontRight;
    private SwerveModule backLeft;
    private SwerveModule backRight;

    // FIXME ik had ff geen zin in een fatsoenlijke getter/setter
    public final PigeonIMU pigeon = new PigeonIMU(PIGEON_IMU);

    //Objects used for heading correction
    Timer timer = new Timer();
    double previousT;
    double offT;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            frontLeftLocation,
            frontRightLocation,
            backLeftLocation,
            backRightLocation
    );

    private final SecondOrderSwerveKinematics secondKinematics = new SecondOrderSwerveKinematics(
            frontLeftLocation,
            frontRightLocation,
            backLeftLocation,
            backRightLocation
    );

    // Odometry
    private SwerveDriveOdometry odometry;

    // Pure Pursuit
    private final AdaptivePurePursuitController aPPController;
    public final Field2d field = new Field2d();

    private SwerveDrivetrainHelper swerveDrivetrainHelper;

    public Drivetrain(){
        name = "Drivetrain";
        subsystemManager = new DrivetrainManager();

        aPPController = new AdaptivePurePursuitController(
                DRIVETRAIN_WIDTH,
                MAX_VELOCITY,
                MIN_VELOCITY,
                MAX_ACCELERATION,
                PATH_END_DISTANCE,
                MAX_LOOKAHEAD,
                MIN_LOOKAHEAD
        );

        subsystemManager.setAPPController(aPPController);
        subsystemManager.setOdometry(odometry);

        initializeSwerveModules();

        odometry = new SwerveDriveOdometry(
                kinematics,
                Rotation2d.fromDegrees(pigeon.getYaw()),
                new SwerveModulePosition[]{
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                }
        );

        swerveDrivetrainHelper = new SwerveDrivetrainHelper(
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                kinematics,
                secondKinematics,
                pigeon
        );
        swerveDrivetrainHelper.setMaxVelocity(MAX_VELOCITY);
        swerveDrivetrainHelper.setFieldRelative(true);

        SmartDashboard.putData("Field", field);
    }

    @Override
    public void onStart(double timestamp) {
        if (DriverStation.isAutonomous()){
            //AUTONOMOUS IS HANDLED BY THE APP CONTROLLER, DO NOTHING HERE
            subsystemManager.setControlState(DrivetrainManager.controlState.DISABLED);
        } else {
            subsystemManager.setControlState(DrivetrainManager.controlState.ENABLED);
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
                swerveDrivetrainHelper.idleSwerveModuleStates();
                break;
            case ENABLED:
                subsystemManager.setCurrentPose(
                        odometry.update(
                                Rotation2d.fromDegrees(pigeon.getYaw()),
                                new SwerveModulePosition[]{
                                    frontLeft.getPosition(),
                                    frontRight.getPosition(),
                                    backLeft.getPosition(),
                                    backRight.getPosition()
                                }
                ));
                if (subsystemManager.getBoost()) {
                    swerveDrivetrainHelper.setMaxVelocity(MAX_VELOCITY_BOOST);
                } else {
                    swerveDrivetrainHelper.setMaxVelocity(MAX_VELOCITY);
                }
                ChassisSpeeds desiredSpeeds = subsystemManager.getDesiredSpeeds();
                ChassisSpeeds correctedSpeeds = correctHeading(desiredSpeeds);
                swerveDrivetrainHelper.updateSwerveModuleStates(correctedSpeeds);
                break;
        }

    }


    @Override
    public void writePeriodicOutputs() {
        SmartDashboard.putNumber("DesiredSpeed vx", subsystemManager.getDesiredSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("DesiredSpeed vy", subsystemManager.getDesiredSpeeds().vyMetersPerSecond);
        SmartDashboard.putNumber("DesiredSpeed omega", subsystemManager.getDesiredSpeeds().omegaRadiansPerSecond);
        SmartDashboard.putNumber("dx", subsystemManager.getCurrentPose().getX());
        SmartDashboard.putNumber("dy", subsystemManager.getCurrentPose().getY());
        SmartDashboard.putNumber("dtheta", subsystemManager.getCurrentPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Pigeon Yaw", pigeon.getYaw());
        SmartDashboard.putNumber("Current drive speed frontleft", frontLeft.getDriveVelocity());
        SmartDashboard.putNumber("Current angle frontleft", frontLeft.getAbsoluteAngle().getRadians());

    }

    @Override
    public void onStop(double timestamp) {
        terminate();
    }

    @Override
    public void zeroSensors() {
        pigeon.setYaw(0);

        subsystemManager.setTargetHeading( new Rotation2d(0) );
        timer.reset();
        timer.start();
        previousT = 0;
        offT = 0;

        resetOdometry(new Pose2d());

        frontLeft.calibrateTurnEncoder();
        frontRight.calibrateTurnEncoder();
        backLeft.calibrateTurnEncoder();
        backRight.calibrateTurnEncoder();
    }

    @Override
    public void terminate() {
        subsystemManager.setControlState(DrivetrainManager.controlState.DISABLED);
    }

    @Override
    public void outputData() {

    }

    public SwerveDriveOdometry getOdometry() {
        return odometry;
    }

    // FIXME nog meer beun zooi
    public void updateCurrentPose() {
        subsystemManager.setCurrentPose(odometry.getPoseMeters());
    }

    /**
     * This function optimizes the chassis speed that is put into the kinematics object to allow the robot to hold its heading
     * when no angular velocity is input. The robot will therefore correct itself when it turns without us telling it to do so.
     *
     * @param desiredSpeed Desired chassis speed that is input by the controller
     * @return {@code correctedChassisSpeed} which takes into account that the robot needs to have the same heading when no rotational speed is input
     */
    private ChassisSpeeds correctHeading(ChassisSpeeds desiredSpeed){

        //Determine time interval
        double currentT = timer.get();
        double dt = currentT - previousT;

        //Get desired rotational speed in radians per second and absolute translational speed in m/s
        double vr = desiredSpeed.omegaRadiansPerSecond;
        double v = Math.sqrt(Math.pow(desiredSpeed.vxMetersPerSecond, 2) + Math.pow(desiredSpeed.vxMetersPerSecond, 2));

        if (vr > 0.01 || vr < -0.01){
            offT = currentT;
            subsystemManager.setTargetHeading( Rotation2d.fromDegrees(pigeon.getYaw()) );
            return desiredSpeed;
        }
        if (currentT - offT < 0.5){
            subsystemManager.setTargetHeading( Rotation2d.fromDegrees(pigeon.getYaw()) );
            return desiredSpeed;
        }
        if (v < 0.05){
            subsystemManager.setTargetHeading( Rotation2d.fromDegrees(pigeon.getYaw()) );
            return desiredSpeed;
        }


        //Determine target and current heading
        subsystemManager.setTargetHeading( subsystemManager.getTargetHeading().plus(new Rotation2d(vr * dt)) );
        Rotation2d currentHeading = Rotation2d.fromDegrees(pigeon.getYaw());

        //Calculate the change in heading that is needed to achieve the target
        Rotation2d deltaHeading = subsystemManager.getTargetHeading().minus(currentHeading);

        if (Math.abs(deltaHeading.getDegrees()) < TURNING_DEADBAND){
            return desiredSpeed;
        }
        double correctedVr = deltaHeading.getRadians() / dt * HEADING_kP;

        previousT = currentT;

        return new ChassisSpeeds(
                desiredSpeed.vxMetersPerSecond,
                desiredSpeed.vyMetersPerSecond,
                correctedVr);
    }

    /**
     * Set the target heading of the drivetrain such that it faces towards the
     * position that is input
     *
     * @param lookAtTrans {@code Translation2d} object that represents the position
     *                                         that the drivetrain will face
     */
    public void lookAtPosition(Translation2d lookAtTrans){
        Translation2d currentTrans =  subsystemManager.getCurrentPose().getTranslation();
        Translation2d deltaTrans = lookAtTrans.minus(currentTrans);

        subsystemManager.setTargetHeading (
                new Rotation2d( Math.atan2(deltaTrans.getY(), deltaTrans.getX()) )
        );
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose){
        subsystemManager.setCurrentPose(pose);
        odometry.resetPosition(
                Rotation2d.fromDegrees(pigeon.getYaw()),
                new SwerveModulePosition[]{
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                },
                pose
        );
    }

    public void initializeSwerveModules() {
        SwerveModulePhysicalConstants physicalConstants = new SwerveModulePhysicalConstants(
                DRIVE_GEAR_RATIO,
                WHEEL_RADIUS,
                TURN_GEAR_RATIO
        );
        physicalConstants.calculateConversionFactors();

        PIDValueContainer TurnPIDValues = new PIDValueContainer(
                TURN_kP,
                TURN_kI,
                TURN_kD
        );
        FFValueContainer TurnFFValues = new FFValueContainer(
                TURN_kS,
                TURN_kV,
                TURN_kA
        );
        PIDValueContainer DrivePIDValues = new PIDValueContainer(
                DRIVE_kP,
                DRIVE_kI,
                DRIVE_kD
        );
        FFValueContainer DriveFFValues = new FFValueContainer(
                DRIVE_kS,
                DRIVE_kV,
                DRIVE_kA
        );

        SwerveTurnHelper frontLeftTurnHelper = new SwerveTurnHelper(
                DT_FRONT_LEFT_TURN_ID,
                DT_FRONT_LEFT_SENSOR_ID,
                DT_FRONT_LEFT_OFFSET,
                TurnPIDValues,
                TurnFFValues
        );
        SwerveDriveHelper frontLeftDriveHelper = new SwerveDriveHelper(
                DT_FRONT_LEFT_DRIVE_ID,
                DT_FRONT_LEFT_INVERTED,
                DrivePIDValues,
                DriveFFValues
        );
        frontLeft = new SwerveModule(
                frontLeftTurnHelper,
                frontLeftDriveHelper,
                physicalConstants
        );

        SwerveTurnHelper frontRightTurnHelper = new SwerveTurnHelper(
                DT_FRONT_RIGHT_TURN_ID,
                DT_FRONT_RIGHT_SENSOR_ID,
                DT_FRONT_RIGHT_OFFSET,
                TurnPIDValues,
                TurnFFValues
        );
        SwerveDriveHelper frontRightDriveHelper = new SwerveDriveHelper(
                DT_FRONT_RIGHT_DRIVE_ID,
                DT_FRONT_RIGHT_INVERTED,
                DrivePIDValues,
                DriveFFValues
        );
        frontRight = new SwerveModule(
                frontRightTurnHelper,
                frontRightDriveHelper,
                physicalConstants
        );

        SwerveTurnHelper backLeftTurnHelper = new SwerveTurnHelper(
                DT_BACK_LEFT_TURN_ID,
                DT_BACK_LEFT_SENSOR_ID,
                DT_BACK_LEFT_OFFSET,
                TurnPIDValues,
                TurnFFValues
        );
        SwerveDriveHelper backLeftDriveHelper = new SwerveDriveHelper(
                DT_BACK_LEFT_DRIVE_ID,
                DT_BACK_LEFT_INVERTED,
                DrivePIDValues,
                DriveFFValues
        );
        backLeft = new SwerveModule(
                backLeftTurnHelper,
                backLeftDriveHelper,
                physicalConstants
        );

        SwerveTurnHelper backRightTurnHelper = new SwerveTurnHelper(
                DT_BACK_RIGHT_TURN_ID,
                DT_BACK_RIGHT_SENSOR_ID,
                DT_BACK_RIGHT_OFFSET,
                TurnPIDValues,
                TurnFFValues
        );
        SwerveDriveHelper backRightDriveHelper = new SwerveDriveHelper(
                DT_BACK_RIGHT_DRIVE_ID,
                DT_BACK_RIGHT_INVERTED,
                DrivePIDValues,
                DriveFFValues
        );
        backRight = new SwerveModule(
                backRightTurnHelper,
                backRightDriveHelper,
                physicalConstants
        );

        frontLeft.setIdleMode(DRIVE_IDLE_MODE);
        frontRight.setIdleMode(DRIVE_IDLE_MODE);
        backLeft.setIdleMode(DRIVE_IDLE_MODE);
        backRight.setIdleMode(DRIVE_IDLE_MODE);

        frontLeft.setDriveCurrentLimit(STALL_LIMIT_DRIVE, FREE_LIMIT_DRIVE);
        frontRight.setDriveCurrentLimit(STALL_LIMIT_DRIVE, FREE_LIMIT_DRIVE);
        backLeft.setDriveCurrentLimit(STALL_LIMIT_DRIVE, FREE_LIMIT_DRIVE);
        backRight.setDriveCurrentLimit(STALL_LIMIT_DRIVE, FREE_LIMIT_DRIVE);

        frontLeft.setTurnCurrentLimit(STALL_LIMIT_TURN, FREE_LIMIT_TURN);
        frontRight.setTurnCurrentLimit(STALL_LIMIT_TURN, FREE_LIMIT_TURN);
        backLeft.setTurnCurrentLimit(STALL_LIMIT_TURN, FREE_LIMIT_TURN);
        backRight.setTurnCurrentLimit(STALL_LIMIT_TURN, FREE_LIMIT_TURN);
    }
}
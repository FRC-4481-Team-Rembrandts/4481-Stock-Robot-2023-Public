package frc.team4481.robot;

import com.revrobotics.CANSparkMax;

public class Constants {
    /* ---------------------------------------- */
    /* LOOPER */
    /* ---------------------------------------- */
    public static final double kLooperDt = 0.01;
    public static double kHIDLooperDt = 0.02;

    /* ---------------------------------------- */
    /* HARDWAREMAP */
    /* ---------------------------------------- */
    /*
     * Sensor Id's
     */
    public static final int PIGEON_IMU = 9;

    /*
     * Swerve module constants
     */
    public static final int DT_FRONT_LEFT_DRIVE_ID = 6;
    public static final int DT_FRONT_LEFT_TURN_ID = 8;
    public static final int DT_FRONT_LEFT_SENSOR_ID = 5;
    public static final int DT_FRONT_LEFT_OFFSET = 311;
    public static final boolean DT_FRONT_LEFT_INVERTED = true;

    public static final int DT_FRONT_RIGHT_DRIVE_ID = 2;
    public static final int DT_FRONT_RIGHT_TURN_ID = 1;
    public static final int DT_FRONT_RIGHT_SENSOR_ID = 0;
    public static final int DT_FRONT_RIGHT_OFFSET = 50;
    public static final boolean DT_FRONT_RIGHT_INVERTED = false;

    public static final int DT_BACK_LEFT_DRIVE_ID = 7;
    public static final int DT_BACK_LEFT_TURN_ID = 5;
    public static final int DT_BACK_LEFT_SENSOR_ID = 4;
    public static final int DT_BACK_LEFT_OFFSET = 40;
    public static final boolean DT_BACK_LEFT_INVERTED = true;

    public static final int DT_BACK_RIGHT_DRIVE_ID = 3;
    public static final int DT_BACK_RIGHT_TURN_ID = 4;
    public static final int DT_BACK_RIGHT_SENSOR_ID = 1;
    public static final int DT_BACK_RIGHT_OFFSET = 255;
    public static final boolean DT_BACK_RIGHT_INVERTED = false;

    /* ---------------------------------------- */
    /* DRIVETRAIN */
    /* ---------------------------------------- */
    public static final double DRIVETRAIN_WHEELBASE_DISTANCE = 0.285; //Wheel to wheel distance / 2 in m
    public static final double DRIVE_GEAR_RATIO = 5.1;                 // standard REV swerve module gear ratio
    public static final double WHEEL_RADIUS = 0.07695 / 2;                   //old value: 0.0381;
    public static final double DRIVETRAIN_WIDTH = 0.66;                 //Drivetrain width in m
    public static final double DRIVE_VELOCITY_CONVERSION_FACTOR = (2 * Math.PI * WHEEL_RADIUS)/(DRIVE_GEAR_RATIO * 60); // RPM to wheel velocity m/s
    public static final double TURN_GEAR_RATIO = 2.89 * 3.61 /14 * 62;
    public static final double TURN_VELOCITY_CONVERSION_FACTOR = 2 * Math.PI / (TURN_GEAR_RATIO * 60); // RPM to radians per second
    public static final double TURN_POSITION_CONVERSION_FACTOR = 2 * Math.PI / TURN_GEAR_RATIO; // rotation to radians per second
    public static final double DRIVE_kP = 0.01;
    public static final double DRIVE_kI = 0;
    public static final double DRIVE_kD = 0;
    public static final int STALL_LIMIT_DRIVE = 28;//in Amps
    public static final int FREE_LIMIT_DRIVE = 28; //in Amps
    public static final int STALL_LIMIT_TURN = 18;//in Amps
    public static final int FREE_LIMIT_TURN = 18; //in Amps
    public static final double HEADING_kP = 0.04; //P constant for heading correction
    public static final double TURNING_DEADBAND = 2; //Margin that the heading correction algorithm has in degrees


    // Feedforward values from SysId
    public static final double DRIVE_kA = 1.2;
    public static final double DRIVE_kS = 0.19043;
    public static final double DRIVE_kV = 3.5;

    public static final double TURN_kA = 0;
    public static final double TURN_kS = 0.2; //0.33035, 2
    public static final double TURN_kV = 0.6; //0.6

    public static final double TURN_kP = 10; //12.5
    public static final double TURN_kI = 0;
    public static final double TURN_kD = 0.4;

    public static final CANSparkMax.IdleMode DRIVE_IDLE_MODE = CANSparkMax.IdleMode.kBrake;


    /* ---------------------------------------- */
    /* PATH PLANNING */
    /* ---------------------------------------- */
    /*
     * Maximum robot acceleration in m/s^2
     * Maximum robot velocity in m/s
     * Minimum robot velocity in m/s
     */
    public static final double MAX_ACCELERATION = 5;
    public static final double MAX_VELOCITY = 2.5; // 3; //3.8
    public static final double MAX_VELOCITY_BOOST = 3.8;
    public static final double MIN_VELOCITY = 0.25;
    public static final double MAX_TURN_VELOCITY = 2*Math.PI;
    public static final double ROTATION_KP = 0.35;

    /*
     * Maximum distance from final point that counts as ended path in m
     */
    public static final double PATH_END_DISTANCE = 0.3;

    /*
     * Minimum lookahead distance in m
     * Maximum lookahead distance in m
     */
    public static final double MIN_LOOKAHEAD = .3;
    public static final double MAX_LOOKAHEAD = 1;

    /* ---------------------------------------- */
    /* CONTROLLERS */
    /* ---------------------------------------- */
    public static final double ORANGE_LEFTSTICK_DEADBAND = 0.15;
    public static final double ORANGE_RIGHTSTICK_DEADBAND = 0.15;
}
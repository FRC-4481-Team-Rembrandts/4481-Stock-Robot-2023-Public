package frc.team4481.lib.swerve;

public class SwerveModulePhysicalConstants
{
    private double drive_gear_ratio;
    private double wheel_radius;
    private double turn_gear_ratio;

    public double drive_velocity_conversion_factor;
    public double drive_position_conversion_factor;
    public double turn_velocity_conversion_factor;
    public double turn_position_conversion_factor;

    public SwerveModulePhysicalConstants(double drive_gear_ratio, double wheel_radius, double turn_gear_ratio)
    {
        this.drive_gear_ratio = drive_gear_ratio;
        this.wheel_radius = wheel_radius;
        this.turn_gear_ratio = turn_gear_ratio;
    }

    public void calculateConversionFactors()
    {
        // RPM to wheel velocity m/s
        drive_velocity_conversion_factor =  (2 * Math.PI * wheel_radius) / (drive_gear_ratio * 60);

        // Rotations to meters
        drive_position_conversion_factor = 2 * Math.PI * wheel_radius / drive_gear_ratio;

        // RPM to radians per second
        turn_velocity_conversion_factor = 2 * Math.PI / (turn_gear_ratio * 60);

        // rotation to radians per second
        turn_position_conversion_factor = 2 * Math.PI / turn_gear_ratio;
    }
}
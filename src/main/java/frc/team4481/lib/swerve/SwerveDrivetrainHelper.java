package frc.team4481.lib.swerve;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveDrivetrainHelper
{
    private SwerveModule frontLeft;
    private SwerveModule frontRight;
    private SwerveModule backLeft;
    private SwerveModule backRight;

    private SwerveDriveKinematics kinematics;
    private SecondOrderSwerveKinematics secondKinematics;

    private PigeonIMU pigeon;

    private double maxVelocity;
    private double maxVelocityBoost;
    private boolean fieldRelative;

    public SwerveDrivetrainHelper(
            SwerveModule frontLeft,
            SwerveModule frontRight,
            SwerveModule backLeft,
            SwerveModule backRight,
            SwerveDriveKinematics kinematics,
            SecondOrderSwerveKinematics secondKinematics,
            PigeonIMU pigeon
            )
    {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.kinematics = kinematics;
        this.secondKinematics = secondKinematics;
        this.pigeon = pigeon;
    }

    public void updateSwerveModuleStates(ChassisSpeeds desiredSpeeds) {
        // Convert robot  relative speeds to field relative speeds if desired
        ChassisSpeeds desiredSpeedsFieldRelative = null;
        if (fieldRelative) {
            desiredSpeedsFieldRelative = desiredSpeeds;
            double xSpeed = desiredSpeeds.vxMetersPerSecond;
            double ySpeed = desiredSpeeds.vyMetersPerSecond;
            double rot = desiredSpeeds.omegaRadiansPerSecond;

            desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(pigeon.getYaw()));
        }

        // Convert chassis speeds to individual module states
        // The second order kinematics will return swerveModuleStates and desired rotational speeds of the turn motors
        // Both the swerveModuleStates and the desired turn velocity are saved in the secondOrderSwerveModuleState object
        SecondOrderSwerveModuleStates secondOrderSwerveModuleStates = secondKinematics.toSwerveModuleState(desiredSpeedsFieldRelative, Rotation2d.fromDegrees(pigeon.getYaw()));
        SwerveModuleState[] swerveModuleStates = secondOrderSwerveModuleStates.getSwerveModuleStates();
        double[] moduleTurnSpeeds = secondOrderSwerveModuleStates.getModuleTurnSpeeds();

        //Desaturate the individual module velocity with the right max velocity
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxVelocity);


        // Assign desired module states to modules
        frontLeft.setDesiredState(swerveModuleStates[0], moduleTurnSpeeds[0]);
        frontRight.setDesiredState(swerveModuleStates[1], moduleTurnSpeeds[1]);
        backLeft.setDesiredState(swerveModuleStates[2], moduleTurnSpeeds[2]);
        backRight.setDesiredState(swerveModuleStates[3], moduleTurnSpeeds[3]);
    }

    /**
     * Sets {@code ServeModule} in cross configuration
     */
    public void idleSwerveModuleStates() {
        // TODO add enum for different idle positions
        int numModules = 4;
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[numModules];
        int[] angles = {45, -45, -45, 45}; // Cross config

        for (int i = 0; i < numModules; i++) {
            Rotation2d angle = Rotation2d.fromDegrees(angles[i]);
            swerveModuleStates[i] = new SwerveModuleState(0, angle);
        }

        // Assign desired module states to modules
        frontLeft.setDesiredState(swerveModuleStates[0], 0);
        frontRight.setDesiredState(swerveModuleStates[1], 0);
        backLeft.setDesiredState(swerveModuleStates[2], 0);
        backRight.setDesiredState(swerveModuleStates[3], 0);
    }

    public void setMaxVelocity(double velocity) {
        maxVelocity = velocity;
    }

    public double getMaxVelocity() {
        return maxVelocity;
    }

    public void setFieldRelative(boolean isFieldRelative) {
        fieldRelative = isFieldRelative;
    }

    public boolean getFieldRelative() {
        return fieldRelative;
    }
}

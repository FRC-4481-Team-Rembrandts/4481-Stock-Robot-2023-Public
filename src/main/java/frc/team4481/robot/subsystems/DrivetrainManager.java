package frc.team4481.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import frc.team4481.lib.path.AdaptivePurePursuitController;
import frc.team4481.lib.subsystems.SubsystemManagerBase;

public class DrivetrainManager extends SubsystemManagerBase {

    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds(0,0,0);
    private controlState currentControlState = controlState.DISABLED;

    private AdaptivePurePursuitController aPPController;
    private SwerveDriveOdometry odometry;
    private Pose2d currentPose;
    private Rotation2d targetHeading;
    private boolean boost;

    public enum controlState {
        DISABLED,
        ENABLED
    }

    public ChassisSpeeds getDesiredSpeeds() {
        return desiredSpeeds;
    }

    public void setDesiredSpeeds(ChassisSpeeds desiredSpeeds) {
        this.desiredSpeeds = desiredSpeeds;
    }

    public void setControlState(controlState pControlState) {
        currentControlState = pControlState;
    }
    public controlState getControlState() {
        return currentControlState;
    }

    public void setAPPController(AdaptivePurePursuitController aPPController) {
        this.aPPController = aPPController;
    }
    public AdaptivePurePursuitController getAPPController() {
        return aPPController;
    }

    public void setOdometry(SwerveDriveOdometry odometry) {
        this.odometry = odometry;
    }
    public SwerveDriveOdometry getOdometry() {
        return odometry;
    }

    public Pose2d getCurrentPose() {
        return currentPose;
    }
    public void setCurrentPose(Pose2d currentPose) {
        this.currentPose = currentPose;
    }

    public Rotation2d getTargetHeading(){ return targetHeading; }
    public void setTargetHeading(Rotation2d targetHeading) { this.targetHeading = targetHeading; }

    public boolean getBoost() { return boost; }
    public void setBoost(boolean boost) {
        this.boost = boost;
    }
}

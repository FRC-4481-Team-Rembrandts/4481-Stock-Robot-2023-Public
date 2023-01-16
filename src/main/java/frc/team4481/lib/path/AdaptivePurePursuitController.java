package frc.team4481.lib.path;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

import static frc.team4481.robot.Constants.*;

/**
 * The AdaptivePurePursuitController class implements the Pure Pursuit algorithm for differential and swerve drive.
 * This algorithm computes a trajectory curve towards the {@code Translation2d} at a set offset
 * from the current {@code Translation2d} of the robot. This offset is called the lookahead distance.
 * <p>
 * This curve is then used in combination with a target velocity to calculate separate target velocities
 * for the left and right sides of the {@code Drivetrain} in meters/second.
 * <p>
 * Before a {@code Trajectory} can be followed, it first has to be set using {@code setNewTrajectory()}.
 * After that, a new {@code DifferentialDriveSpeeds} or {@code ChassisSpeeds} has to be calculated every cycle
 * for optimal robot performance.
 * <p>
 * A path is finished when the closest point on the path to the robot is the last point in the path and the distance
 * is smaller than {@code PATH_END_DISTANCE}.
 *
 * @see AdaptivePurePursuitController#setNewTrajectory(String, boolean) setNewTrajectory
 * @see AdaptivePurePursuitController#getTargetPose2d(Pose2d)  getTargetPose2d
 * @see AdaptivePurePursuitController#getChassisSpeedsForTargetDelta(Pose2d, Pose2d, double) getChassisSpeedsForTargetDelta 
 */

public class AdaptivePurePursuitController {
    private final TrajectoryHandler trajectoryHandler;

    private PathPlannerTrajectory currentTrajectory;
    private ArrayList<PathPlannerTrajectory.PathPlannerState> trajectoryStates;

    private boolean isPathReversed = false;
    private boolean pathFinished = false;

    private final double DRIVETRAIN_WIDTH;
    private final double MAX_VELOCITY;
    private final double MIN_VELOCITY;
    private final double MAX_ACCELERATION;
    private final double PATH_END_DISTANCE;
    private final double MAX_LOOKAHEAD;
    private final double MIN_LOOKAHEAD;

    /**
     * Creates a new {@code AdaptivePurePursuitController} with the following parameters.
     *
     * @param drivetrainWidth center-to-center distance between the left and right wheels side in meters
     * @param maxVelocity maximum velocity of the drivetrain
     * @param minVelocity minimum velocity when the robot is not commanded to yield
     * @param maxAcceleration maximum acceleration of the drivetrain
     * @param pathEndDistance if the robot is within this distance (m) of the end of the path, it is considered finished
     * @param maxLookahead maximum radius of the lookahead circle in meters
     * @param minLookahead minimum radius of the lookahead circle in meters
     */
    public AdaptivePurePursuitController(
            double drivetrainWidth,
            double maxVelocity,
            double minVelocity,
            double maxAcceleration,
            double pathEndDistance,
            double maxLookahead,
            double minLookahead
    ) {
        DRIVETRAIN_WIDTH = drivetrainWidth;
        MAX_VELOCITY = maxVelocity;
        MIN_VELOCITY = minVelocity;
        MAX_ACCELERATION = maxAcceleration;
        PATH_END_DISTANCE = pathEndDistance;
        MAX_LOOKAHEAD = maxLookahead;
        MIN_LOOKAHEAD = minLookahead;

        trajectoryHandler = TrajectoryHandler.getInstance();
    }

    /**
     * Sets {@code Trajectory} to follow from name as defined in Path Planner
     *
     * @param pathName      Name of the path according to Path Planner
     * @param isReversed    If the path should be driven backwards
     * @return {@code Trajectory} to follow
     */
    public PathPlannerTrajectory setNewTrajectory(String pathName, boolean isReversed) {
        return setNewTrajectory(pathName, isReversed, MAX_VELOCITY, MAX_ACCELERATION);
    }

    /**
     * Sets {@code Trajectory} to follow from name as defined in Path Planner
     *
     * @param pathName      Name of the path according to Path Planner
     * @param isReversed    If the path should be driven backwards
     * @return {@code Trajectory} to follow
     */
    public PathPlannerTrajectory setNewTrajectory(
            String pathName,
            boolean isReversed,
            double maxVelocity,
            double maxAcceleration
    ) {
        currentTrajectory = trajectoryHandler.newTrajectory(pathName, isReversed, maxVelocity, maxAcceleration);

        // Store states in ArrayList
        trajectoryStates = trajectoryHandler.getStatesFromTrajectory();

        // Reset pathFinished
        pathFinished = false;

        // Set path reversed state
        isPathReversed = isReversed;

        return currentTrajectory;
    }

    /**
     * Calculates target {@code Pose2d} for current {@code Pose2d} using the Pure Pursuit algorithm.
     *
     * @see Pose2d
     *
     * @param currentPose Current pose of the robot
     * @return Target Pose2d of the robot according to the Pure Pursuit algorithm
     */
    public Pose2d getTargetPose2d(Pose2d currentPose) {
        Translation2d translation = currentPose.getTranslation();

        // Calculate index of the closest point in path
        int closestPosIndex = getClosestPosIndexInPath(translation);

        double targetVelocity = getTargetVelocity(trajectoryStates,closestPosIndex);

        // Determine point to travel to with lookahead distance
        int indexToTarget = getClosestPosIndexToLookahead(
                trajectoryStates,
                closestPosIndex,
                currentPose.getTranslation(),
                getLookaheadForVelocity(targetVelocity)
        );

        PathPlannerTrajectory.PathPlannerState targetState = trajectoryStates.get(indexToTarget);
        Pose2d targetPose = targetState.poseMeters;
        Rotation2d holoRotation = targetState.holonomicRotation;

        // Correct rotation to holonomic
        Pose2d modifiedTarget = new Pose2d(targetPose.getTranslation(), holoRotation);

        // Get last point in path
        Translation2d finalPosition = currentTrajectory.getEndState().poseMeters.getTranslation();

        // Check if path is finished
        double targetDistance = finalPosition.getDistance(currentPose.getTranslation());

        // Path is finished if target is closer than 30 cm to current position
        pathFinished = targetDistance < PATH_END_DISTANCE;

        return modifiedTarget;
    }

    /**
     * Calculates the ChassisSpeeds for a given Pose2d delta based on the provided target velocity
     *
     * @param currentPose       current {@code Pose2d} of the robot in field space
     * @param targetPose        target {@code Pose2d} of the robot in field space
     * @param targetVelocity    velocity in m/s at which to drive
     * @return target ChassisSpeeds for a holonomic drivetrain
     */
    public ChassisSpeeds getChassisSpeedsForTargetDelta(Pose2d currentPose, Pose2d targetPose, double targetVelocity) {
        Translation2d deltaTrans = targetPose.getTranslation().minus(currentPose.getTranslation());
        Rotation2d deltaRot = targetPose.getRotation().minus(currentPose.getRotation());

        // calculate x and y components of velocity in field space
        double vx = targetVelocity * deltaTrans.getX() / deltaTrans.getNorm();
        double vy = targetVelocity * deltaTrans.getY() / deltaTrans.getNorm();

        // calculate angular velocity
        double vr = deltaRot.getRadians() * ROTATION_KP * MAX_TURN_VELOCITY;

        return new ChassisSpeeds(vx, vy, vr);
    }

    /**
     * Calculates the {@code DifferentialDriveWheelSpeeds} for a given
     * {@code Pose2d} delta based on the provided target velocity
     *
     * @param currentPose       current {@code Pose2d} of the robot in field space
     * @param targetPose        target {@code Pose2d} of the robot in field space
     * @param targetVelocity    velocity in m/s at which to drive
     * @return separate targets for the left and right side of the drivetrain in m/s
     */
    public DifferentialDriveWheelSpeeds getDifferentialSpeedsForTargetDelta(
            Pose2d currentPose,
            Pose2d targetPose,
            double targetVelocity
    ) {
        // Check if path is finished
        double distanceToEnd =
                currentPose.getTranslation().getDistance(currentTrajectory.getEndState().poseMeters.getTranslation());

        if (distanceToEnd < PATH_END_DISTANCE){
            pathFinished = true;
            return new DifferentialDriveWheelSpeeds(0,0);
        }

        // if not, calculate by normal means
        double relativeHorizontal = getRelativeHorizontal(currentPose, targetPose.getTranslation());
        double curvature = getCurvature(currentPose, targetPose.getTranslation(), relativeHorizontal);

        return calculateDifferentialWheelSpeeds(targetVelocity, curvature);
    }

    /**
     * Finds out if the robot has reached the end of the current path
     *
     * @see Trajectory
     *
     * @return If the path is finished or not
     */
    public boolean isPathFinished() {
        return pathFinished;
    }

    /**
     * Gets the current {@code Trajectory} that the robot will follow
     *
     * @return {@code Trajectory} that the robot will follow
     */
    public Trajectory getCurrentTrajectory() {
        return currentTrajectory;
    }

    /**
     * Get whether the path is reversed.
     *
     * @return whether the path is reversed
     */
    public boolean getPathReversed() {
        return isPathReversed;
    }

    /**
     * Calculates index of closest {@code Translation2d} in {@code Path} compared to robot {@code Translation2d}
     *
     * @param currentPos Current robot {@code Translation2d}
     * @return Index of closest {@code Translation2d} in {@code currentPath}
     */
    private int getClosestPosIndexInPath(Translation2d currentPos) {
        int smallestDistanceIndex = 0;
        // Init as infinity so that there is always a smaller distance somewhere in the array
        double smallestDistance = Double.POSITIVE_INFINITY;

        // Loop over every Position in path to check closest
        for (int i = 0; i < trajectoryStates.size(); i++) {
            double currentDistance = currentPos.getDistance(trajectoryStates.get(i).poseMeters.getTranslation());

            // Set smallest index to current index if distance is smaller
            if (currentDistance < smallestDistance) {
                smallestDistance = currentDistance;
                smallestDistanceIndex = i;
            }
        }

        return smallestDistanceIndex;
    }

    /**
     * Calculates index of {@code Translation2d} in {@code Path} closest to lookahead distance.
     *
     * @param stateArray            The array of states that make up the current {@code Trajectory}
     * @param closestPathPosIndex   Index of closest {@code Translation2d} to robot from {@code Path}
     * @param robotPos              Current robot {@code Translation2d}
     * @param lookahead             Radius of the lookahead circle in meters
     * @return Index of {@code Translation2d} closest to lookahead distance
     */
    private int getClosestPosIndexToLookahead(
            ArrayList<PathPlannerTrajectory.PathPlannerState> stateArray,
            int closestPathPosIndex,
            Translation2d robotPos,
            double lookahead
    ) {
        int currentIndex = closestPathPosIndex;
        double currentDistance = robotPos.getDistance(stateArray.get(currentIndex).poseMeters.getTranslation());

        while (currentDistance < lookahead && currentIndex < stateArray.size() - 1) {
            currentDistance = robotPos.getDistance(stateArray.get(currentIndex).poseMeters.getTranslation());
            currentIndex++;
        }

        return currentIndex;
    }

    /**
     * Calculates a lookahead distance in meters based on a current lookahead distance in m/s
     *
     * @param velocity Current robot velocity in m/s
     * @return A lookahead distance in m based on current velocity
     */
    private double getLookaheadForVelocity(double velocity) {
        /* Algorithm based on code by FRC 254 */
        double deltaLookahead = MAX_LOOKAHEAD - MIN_LOOKAHEAD;
        double deltaVelocity = MAX_VELOCITY - MIN_VELOCITY;
        double lookahead = deltaLookahead * (velocity - MIN_VELOCITY) / deltaVelocity + MIN_LOOKAHEAD;

        // Clamp calculated lookahead between min and max lookahead
        return Double.isNaN(lookahead) ? MIN_LOOKAHEAD : Math.max(MIN_LOOKAHEAD, Math.max(MAX_LOOKAHEAD, lookahead));
    }

    /**
     * Gets relative horizontal distance to target point in robot space
     *
     * @param robotPose        Current {@code Pose2d} of the robot
     * @param lookaheadPoint   {@code Translation2d} on the current Trajectory closest to the lookahead distance
     * @return Relative horizontal distance of the robot in meters
     */
    private double getRelativeHorizontal(Pose2d robotPose, Translation2d lookaheadPoint){
        double a = -Math.tan(robotPose.getRotation().getRadians());
        double b = 1;
        double c = Math.tan(robotPose.getRotation().getRadians()) * robotPose.getX() - robotPose.getY();

        return Math.abs(a * lookaheadPoint.getX() + b * lookaheadPoint.getY() + c) / Math.hypot(a, b);
    }

    /**
     * Calculates the curvature of the arc through the robot position and the target point
     *
     * @param robotPose            Current {@code Pose2d} of the robot
     * @param lookaheadPoint       {@code Translation2d} on the current Trajectory closest to the lookahead distance
     * @param relativeHorizontal   Relative horizontal distance from robot to target in robot space in meters
     * @return Curvature of arc between robot and target point
     */
    private double getCurvature(Pose2d robotPose, Translation2d lookaheadPoint, double relativeHorizontal){
        double lookaheadDistance = robotPose.getTranslation().getDistance(lookaheadPoint);

        double curvature = (2 * relativeHorizontal) / Math.pow(lookaheadDistance, 2);

        double side = Math.signum(
                Math.sin(
                        robotPose.getRotation().getRadians()) * (lookaheadPoint.getX() - robotPose.getX()) -
                        Math.cos(robotPose.getRotation().getRadians()) * (lookaheadPoint.getY() - robotPose.getY())
        );

        return side * curvature;
    }

    /**
     * Calculates the {@code DifferentialDriveWheelSpeeds} based on target velocity and curvature
     * of arc through robot position and lookahead point
     *
     * @param targetVelocity   Target velocity of the robot in m/s
     * @param curvature        Curvature of arc through robot position and lookahead point
     * @return Target velocities for left and right parts of the drivetrain in m/s
     */
    private DifferentialDriveWheelSpeeds calculateDifferentialWheelSpeeds(double targetVelocity, double curvature) {
        return new DifferentialDriveWheelSpeeds(
                targetVelocity * (2 + curvature * DRIVETRAIN_WIDTH) / 2,
                targetVelocity * (2 - curvature * DRIVETRAIN_WIDTH) / 2
        );
    }

    /**
     * Returns the target velocity at the closest point from the robot in m/s
     *
     * @param stateArray        Array of states that make up the current trajectory
     * @param pClosestPosIndex  Array index of the {@code Trajectory.State} closest to the robot
     * @return Target velocity in m/s at the closest point from the robot
     */
    private double getTargetVelocity(
            ArrayList<PathPlannerTrajectory.PathPlannerState> stateArray,
            int pClosestPosIndex
    ) {
        return Math.abs(stateArray.get(pClosestPosIndex).velocityMetersPerSecond);
    }

    /**
     * Gets the velocity setpoint for the closest {@code Pose2d} in the Path as calculated by PathPlanner 2.
     *
     * @param currentPose current {@code Pose2d} of the robot.
     * @return drivetrain velocity setpoint in m/s.
     */
    public double getTargetVelocityForCurrentPose(Pose2d currentPose) {
        int index = getClosestPosIndexInPath(currentPose.getTranslation());
        return getTargetVelocity(TrajectoryHandler.getInstance().getStatesFromTrajectory(), index);
    }
}

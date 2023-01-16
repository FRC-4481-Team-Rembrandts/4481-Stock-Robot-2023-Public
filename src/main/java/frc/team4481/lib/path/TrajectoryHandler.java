package frc.team4481.lib.path;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import java.util.ArrayList;

/**
 * Utility class to load and transform a {@code Trajectory} such that it can be used by an
 * {@code AdaptivePurePursuitController}. This class might be useful for other tools as well
 * but these are not these are currently not supported.
 */
public class TrajectoryHandler {
    private static TrajectoryHandler instance;

    private PathPlannerTrajectory currentTrajectory;
    public boolean hasInitialPose = false;

    private TrajectoryHandler() {}

    /**
     * Gets the {@code TrajectoryHandler} instance.
     *
     * @return singleton instance of the {@code TrajectoryHandler}
     */
    public static TrajectoryHandler getInstance() {
        if (instance == null)
            instance = new TrajectoryHandler();

        return instance;
    }

    /**
     * Transforms a Path Planner 2 path into a {@code PathPlannerTrajectory}
     *
     * @param pathName      Filename of the path minus file extension
     * @param isReversed  Should the robot follow the path in reverse
     * @param maxVelocity Maximum robot velocity on the path in m/s
     * @param maxAcceleration Maximum robot acceleration on the path in m/s^2
     * @return A {@code PathPlannerTrajectory} of the current path
     */
    public PathPlannerTrajectory newTrajectory (
            String pathName,
            boolean isReversed,
            double maxVelocity,
            double maxAcceleration
    ) {
        currentTrajectory = PathPlanner.loadPath(pathName, maxVelocity, maxAcceleration, isReversed);
        return currentTrajectory;
    }

    /**
     * Gets the current Path Planner Trajectory.
     *
     * @return current Path Planner trajectory
     */
    public PathPlannerTrajectory getCurrentTrajectory() {
        return currentTrajectory;
    }

    /**
     * Constructs a {@code List} of {@code PathPlannerStates} from the current trajectory.
     *
     * @return {@code ArrayList} of states in the trajectory.
     */
    public ArrayList<PathPlannerTrajectory.PathPlannerState> getStatesFromTrajectory() {
        // Prune the first entry since it has a target velocity of 0
        // This would result in a robot that would never move
        int i = 1;
        ArrayList<PathPlannerTrajectory.PathPlannerState> states = new ArrayList<>();

        // Add the rest of the states to a new arrayList
        do {
            states.add(currentTrajectory.getState(i));
            i++;
        } while (!currentTrajectory.getState(i).equals(currentTrajectory.getEndState()));

        return states;
    }
}

package frc.robot.lib;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import frc.robot.Config;
import frc.robot.lib.log.Log;

/**
 * Trajectory routines for driving.
 */
public class TrajectoryLib {
    private static final Path cachedTrajectoryPath =
            Paths.get(System.getProperty("user.home"), "paths");

    private static final Path deployedTrajectoryPath =
            Paths.get(System.getProperty("user.home"), "deploy", "pathweaver", "output");

    public static Trajectory getTrajectory(String filename) throws IOException {
        var path = deployedTrajectoryPath.resolve(filename);
        var trajectory = TrajectoryUtil.fromPathweaverJson(path);
        Log.info("Drivebase", "Successfully read " + path.toString());
        return trajectory;
    }

    /**
     * Returns a trajectory by first checking for any cached trajectories in the deploy
     * directory.
     * If it doesn't already exist, generate a trajectory then export it.
     * 
     * This should only be used for unit tests.
     */
    public static Trajectory generateTrajectory(Pose2d start,
            List<Translation2d> interiorWaypoints,
            Pose2d end, boolean forward, boolean relative, Path path) {

        int hash = Arrays.deepHashCode(new Object[] {start, interiorWaypoints, end, forward});
        String trajectoryJSON = String.valueOf(hash) + ".wpilib.json";
        Path trajectoryPath = path.resolve(trajectoryJSON);

        try {
            Files.createDirectories(trajectoryPath.getParent());
            return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException e) {
            Log.warning("Drivebase",
                    "Cached trajectory file not found: Generating and caching spline.");
        }

        // Build the trajectory on start so that it's ready when needed.
        // Create config for trajectory
        TrajectoryConfig config =
                new TrajectoryConfig(Config.drivebase.trajectory.maxSpeedMetersPerSecond,
                        Config.drivebase.trajectory.maxAccelerationMetersPerSecondSquared)
                                // Add kinematics to ensure max speed is actually obeyed
                                .setKinematics(Config.drivebase.trajectory.driveKinematics)
                                // Apply the voltage constraint
                                .addConstraint(
                                        Config.drivebase.trajectory.autoVoltageConstraint)
                                .setReversed(!forward);

        // An example trajectory to follow. All units in meters.
        long t = System.currentTimeMillis();
        Trajectory trajectory =
                TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);

        try {
            TrajectoryUtil.toPathweaverJson(trajectory, trajectoryPath);
            Log.info("Drivebase",
                    "Trajectory Generator: took %d milliseconds to generate and write this spline to file\n",
                    System.currentTimeMillis() - t);
        } catch (IOException e) {
            Log.exception("Drivebase", "Failed to write trajectory file", e);
        }

        return trajectory;
    }

    /**
     * Returns a trajectory by first checking for any cached trajectories in the deploy
     * directory.
     * If it doesn't already exist, generate a trajectory then export it.
     */
    public static Trajectory generateTrajectory(Pose2d start,
            List<Translation2d> interiorWaypoints,
            Pose2d end, boolean forward, boolean relative) {
        Log.info("Drivebase", "Generating trajectory into folder %s", cachedTrajectoryPath);
        return generateTrajectory(start, interiorWaypoints, end, forward, relative,
                cachedTrajectoryPath);
    }
}

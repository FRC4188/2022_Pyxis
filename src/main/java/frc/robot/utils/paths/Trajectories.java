package frc.robot.utils.paths;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants;

public final class Trajectories {
    private static final TrajectoryConfig autoConfig = Constants.drive.auto.CONFIG;

    public static class simple {
        public static final Trajectory straight = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(),
                new Pose2d(5.0, 0, new Rotation2d())
            ), autoConfig);
    }

    public static class twoball {
        public static final Trajectory first = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(),
                new Pose2d(2.2, 0.17, Rotation2d.fromDegrees(24.13))
            ), autoConfig
        );

        public static final Trajectory second = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(2.2, 0.17, Rotation2d.fromDegrees(-81.96)),
                new Pose2d(2.74, -0.95, Rotation2d.fromDegrees(-81.96)),
                new Pose2d(2.32, -2.56, Rotation2d.fromDegrees(-203.03))
            ), autoConfig);
    }
}

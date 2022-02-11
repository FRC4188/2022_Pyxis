package frc.robot.utils.paths;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants;
import frc.robot.Constants.drive.auto;

public final class Trajectories {
    private static final TrajectoryConfig autoConfig = Constants.drive.auto.CONFIG;

    public static class simple {
        public static final Trajectory straight = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(),
                new Pose2d(3.0, 0, new Rotation2d())
            ), autoConfig);
    }

    public static class oneball {
        public static final Trajectory first = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(2.08, -0.66, Rotation2d.fromDegrees(-37.4))
            ), autoConfig);
    }

    public static class twoball {
        public static final Trajectory first = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(),
                new Pose2d(2.08, -0.66, Rotation2d.fromDegrees(-37.4))
            ), autoConfig
        );

        public static final Trajectory second = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(2.2, 0.17, Rotation2d.fromDegrees(-81.96)),
                new Pose2d(2.74, -0.95, Rotation2d.fromDegrees(-81.96)),
                new Pose2d(2.32, -2.56, Rotation2d.fromDegrees(-203.03))
            ), autoConfig);
    }

    public static class threeball {
        public static final Trajectory toFirst = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(2.04, -0.3, Rotation2d.fromDegrees(-32.4))
            ), autoConfig);

        public static final Trajectory toSecond = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(2.04, -0.3, Rotation2d.fromDegrees(-32.4)),
                new Pose2d(2.01, 1.45, Rotation2d.fromDegrees(95.71)),
                new Pose2d(1.74, 3.41, Rotation2d.fromDegrees(157.37))
            ), autoConfig);
        
        public static final Trajectory toThird = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(1.74, 3.41, Rotation2d.fromDegrees(157.37)),
                new Pose2d(-0.53, 3.53, Rotation2d.fromDegrees(154.38)),
                new Pose2d(-1.14, 3.83, Rotation2d.fromDegrees(147.08))
            ), autoConfig);
    }

    public static final class fiveball {
        public static final Trajectory first = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(),
                new Pose2d(2.2, 0.17, Rotation2d.fromDegrees(24.13))
            ), autoConfig
        );

        public static final Trajectory second = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(2.2, 0.17, Rotation2d.fromDegrees(-81.96)),
                new Pose2d(1.37, -1.48, Rotation2d.fromDegrees(-73.65)),
                new Pose2d(1.54, -2.39, Rotation2d.fromDegrees(-48.03)),
                new Pose2d(1.91, -4.95, Rotation2d.fromDegrees(-103.1))
            ), autoConfig);

        public static final Trajectory terminal = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(1.91, -4.95, Rotation2d.fromDegrees(-52.87)),
                new Pose2d(3.19, -6.21, Rotation2d.fromDegrees(-22.98))
            ), autoConfig);
        
        public static final Trajectory lastShoot = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(3.19, -6.21, Rotation2d.fromDegrees(130.74)),
                new Pose2d(1.54, -4.28, Rotation2d.fromDegrees(136.45))
            ), autoConfig);
    }
}
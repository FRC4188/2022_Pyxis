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
                new Pose2d(0.98, 0.0, Rotation2d.fromDegrees(0.0))
            ), autoConfig
        );

        public static final Trajectory terminal1 = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0.98, 0.0, Rotation2d.fromDegrees(-96.0)),
                new Pose2d(0.88, -2.46, Rotation2d.fromDegrees(-90.0)),
                new Pose2d(0.1, -5.73, Rotation2d.fromDegrees(-50.0)),
                new Pose2d(0.45, -5.9, Rotation2d.fromDegrees(-50.0))
            ), autoConfig);
        
        public static final Trajectory terminal2 = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0.45, -5.9, Rotation2d.fromDegrees(130.0)),
                new Pose2d(0.15, -5.78, Rotation2d.fromDegrees(130.0))
            ), autoConfig
        );
        
        public static final Trajectory shoot2 = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0.52, -6.02, Rotation2d.fromDegrees(130.0)),
                new Pose2d(0.63, -3.53, Rotation2d.fromDegrees(119.9))
            ), autoConfig);
        
        public static final Trajectory shoot3 = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0.63, -3.53, Rotation2d.fromDegrees(119.9)),
                new Pose2d(0.08, -2.93, Rotation2d.fromDegrees(116.0))
            ), autoConfig);
    }
}

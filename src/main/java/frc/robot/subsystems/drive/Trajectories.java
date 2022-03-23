package frc.robot.subsystems.drive;

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
        public static final Trajectory toFirst = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(0.87, 0.0, Rotation2d.fromDegrees(0.0))
            ), autoConfig);
    }

    public static class fourball {
        public static final Trajectory toFirst = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(0.87, 0.0, Rotation2d.fromDegrees(0.0))
            ), autoConfig);

        public static final Trajectory toSecond = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0.87, 0.0, Rotation2d.fromDegrees(92.81)),
                new Pose2d(0.69, 5.65, Rotation2d.fromDegrees(85.58))
            ), autoConfig);
        
        public static final Trajectory toThird = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0.69, 5.65, Rotation2d.fromDegrees(-106.2)),
                new Pose2d(-0.6, 3.0, Rotation2d.fromDegrees(-168.92))
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
                new Pose2d(0.86, -2.38, Rotation2d.fromDegrees(-90.0)),
                new Pose2d(0.33, -6.05, Rotation2d.fromDegrees(-46.96))
            ), autoConfig
            );
        
        public static final Trajectory terminal2 = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0.33, -6.05, Rotation2d.fromDegrees(130.0)),
                new Pose2d(-1.67, -4.18, Rotation2d.fromDegrees(-45.41 + 180.0))
            ), new TrajectoryConfig(4.0, 1.0)
            );
        
        public static final Trajectory shoot2 = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(-1.67, -4.18, Rotation2d.fromDegrees(54.18)),
                new Pose2d(-1, -3.15, Rotation2d.fromDegrees(54.18)),
                new Pose2d(-0.45, -3.0, Rotation2d.fromDegrees(53.45))
            ), autoConfig
            );

        
        public static final Trajectory shoot3 = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(-1, -3.15, Rotation2d.fromDegrees(54.18)),
                new Pose2d(-0.26, -2.71, Rotation2d.fromDegrees(53.45))
            ), autoConfig);

    }
}

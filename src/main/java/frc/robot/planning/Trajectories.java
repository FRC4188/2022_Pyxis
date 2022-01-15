package frc.robot.planning;

import java.util.Collection;
import java.util.List;

import org.ejml.dense.row.linsol.AdjustableLinearSolver_DDRM;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.spline.Spline.ControlVector;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import frc.robot.subsystems.drive.Swerve;

public class Trajectories {


    public static class Auto1 {
        public static Trajectory line = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(),
                new Pose2d(1, 0, new Rotation2d()),
                new Pose2d(2, 0, new Rotation2d())
            ),
        new TrajectoryConfig(2.0, 2.5).addConstraint(new CentripetalAccelerationConstraint(2.5)));

        public static Trajectory curve = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(),
                new Pose2d(1.0, 0, new Rotation2d(-Math.PI / 8.0)),
                new Pose2d(1, 1, new Rotation2d(-Math.PI / 4.0))
            ),
        new TrajectoryConfig(2.0, 2.5).addConstraint(new CentripetalAccelerationConstraint(2.5)));

        public static Trajectory sCurve = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(),
                new Pose2d(1.5, 1, new Rotation2d()),
                new Pose2d(2.5, 0, new Rotation2d()),
                new Pose2d(3, 2, new Rotation2d())
            ),
        new TrajectoryConfig(2.0, 2.5).addConstraint(new CentripetalAccelerationConstraint(2.5)));
    }

    public static class trench {
        
        public static Rotation2d[] headings = {
            new Rotation2d(),
            new Rotation2d(Math.PI / 8.0),
            new Rotation2d(-Math.PI)
        };

        private static double adjustment = -0.15;

        public static Trajectory down = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(),
                new Pose2d(1.78, 1.63 + adjustment, new Rotation2d()),
                new Pose2d(5.5, 1.63 + adjustment, new Rotation2d())
            ),
            new TrajectoryConfig(4.0, 3.0)
                .addConstraint(new CentripetalAccelerationConstraint(3.0))
                .setEndVelocity(0.5)
        );

        public static Trajectory ball1 = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(5.5, 1.63 + adjustment, new Rotation2d()),
                new Pose2d(7.0, 1.53 + adjustment, new Rotation2d())
            ),
            new TrajectoryConfig(4.0, 3.0)
                .addConstraint(new CentripetalAccelerationConstraint(3.0))
        );

        public static Trajectory ball2 = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(7.0, 1.53 + adjustment, new Rotation2d(-Math.PI / 2.0)),
                new Pose2d(7.0, 1.7 + adjustment, new Rotation2d(-Math.PI / 2.0))
            ),
            new TrajectoryConfig(4.0, 3.0)
                .addConstraint(new CentripetalAccelerationConstraint(3.0))
        );

        public static Trajectory back = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(5.0, 1.7 + adjustment, new Rotation2d(-Math.PI)),
                new Pose2d(5.0, 1.63 + adjustment, new Rotation2d(-Math.PI)),
                new Pose2d(0.0, 0.0, new Rotation2d(-Math.PI))
            ),
        new TrajectoryConfig(4.0, 3.0).addConstraint(new CentripetalAccelerationConstraint(3.0)));
    }

    public static class middle {

        public static Rotation2d[] headings = {
            new Rotation2d(),
            new Rotation2d(Math.PI / 8.0),
            new Rotation2d(-Math.PI / 8.0),
            Rotation2d.fromDegrees(-95.0),
            Rotation2d.fromDegrees(-155.0),
            new Rotation2d(Math.PI)
        };

        private static double adjustment = 0.1;

        public static Trajectory down = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(),
                new Pose2d(1.78, 1.63 + adjustment, new Rotation2d()),
                new Pose2d(5.5, 1.63 + adjustment, new Rotation2d())
            ),
            new TrajectoryConfig(4.0, 3.3)
                .addConstraint(new CentripetalAccelerationConstraint(3.3))
                .addConstraint(new SwerveDriveKinematicsConstraint(Swerve.getInstance().getKinematics(), 4.0))
        );

        public static Trajectory ball1 = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(5.5, 1.63 + adjustment, new Rotation2d()),
                new Pose2d(6.25, 1.53 + adjustment, new Rotation2d())
            ),
            new TrajectoryConfig(4.0, 3.3)
                .addConstraint(new CentripetalAccelerationConstraint(3.3))
                .addConstraint(new SwerveDriveKinematicsConstraint(Swerve.getInstance().getKinematics(), 4.0))
        );

        public static Trajectory ball2 = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(6.25, 1.53 + adjustment, new Rotation2d(Math.PI / 2.0)),
                new Pose2d(6.25, 1.7 + adjustment, new Rotation2d(Math.PI / 2.0))
            ),
            new TrajectoryConfig(4.0, 3.3)
                .addConstraint(new CentripetalAccelerationConstraint(3.3))
                .addConstraint(new SwerveDriveKinematicsConstraint(Swerve.getInstance().getKinematics(), 4.0))
        );

        public static Trajectory shoot1 = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(6.25, 1.7 + adjustment, new Rotation2d(-Math.PI)),
                new Pose2d(4.5, 1.63, new Rotation2d(-Math.PI)),
                new Pose2d(3.77, 0.1, new Rotation2d(-Math.PI))
            ), 
            new TrajectoryConfig(4.0, 3.3)
                .addConstraint(new CentripetalAccelerationConstraint(3.3))
                .addConstraint(new SwerveDriveKinematicsConstraint(Swerve.getInstance().getKinematics(), 4.0))
        );

        public static Trajectory mid1 = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(3.77, 0.1, new Rotation2d()),
                new Pose2d(3.46, -1.9, Rotation2d.fromDegrees(-155.0)),
                new Pose2d(2.8, -1.95, Rotation2d.fromDegrees(-155.0))
            ), 
            new TrajectoryConfig(4.0, 3.3)
                .addConstraint(new CentripetalAccelerationConstraint(3.3))
                .addConstraint(new SwerveDriveKinematicsConstraint(Swerve.getInstance().getKinematics(), 4.0))
        );

        public static Trajectory mid2 = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(2.8, -1.95, Rotation2d.fromDegrees(25.0)),
                new Pose2d(3.81, -1.62, Rotation2d.fromDegrees(35.0)),
                new Pose2d(3.71, -0.82, Rotation2d.fromDegrees(-149.0)),
                new Pose2d(2.7, -1.2, Rotation2d.fromDegrees(-149.0))
            ),
            new TrajectoryConfig(4.0, 3.3)
                .addConstraint(new CentripetalAccelerationConstraint(3.3))
                .addConstraint(new SwerveDriveKinematicsConstraint(Swerve.getInstance().getKinematics(), 4.0))
        );

        public static Trajectory shoot2 = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(2.7, -1.2, new Rotation2d(-Math.PI / 2.0)),
                new Pose2d(3.2, 0.0, new Rotation2d(-Math.PI / 2.0))
            ),
            new TrajectoryConfig(4.0, 3.3)
                .addConstraint(new CentripetalAccelerationConstraint(3.3))
                .addConstraint(new SwerveDriveKinematicsConstraint(Swerve.getInstance().getKinematics(), 4.0))
        );
    }
/*
    public static class steal {
        private static ControlVectorList collectPath = new ControlVectorList(
            List.of(
                new ControlVector(new double[] {3.774, 3.774 + 3.048}, new double[] {-7.459, -7.459}),
                new ControlVector(new double[] {5.265, 5.265 + 0.302}, new double[] {-7.862, -7.862 - 0.040}),
                new ControlVector(new double[] {6.192, 6.192 + 0.02}, new double[] {-7.499, -7.499 + 1.31}),
                new ControlVector(new double[] {5.345, 5.345 + -0.604}, new double[] {-6.855, -6.855 + 0.463}),
                new ControlVector(new double[] {4.701, 4.701 + -0.766}, new double[] {-5.344, -5.344 + 3.244})
            )
        );
        
        public static Trajectory collect = TrajectoryGenerator.generateTrajectory(
            collectPath,
            new TrajectoryConfig(2.0, 0.5)
        );
    }*/
}

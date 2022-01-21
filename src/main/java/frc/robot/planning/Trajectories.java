package frc.robot.planning;

import java.util.Collection;
import java.util.List;

import org.ejml.dense.row.linsol.AdjustableLinearSolver_DDRM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
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

    public static class Total {
        public static Trajectory toFirst = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(), //start
                new Pose2d() //first ball
            ), new TrajectoryConfig(3.0, 1.0)
        );

        public static Trajectory toSecond = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(), //end of toFirst
                new Pose2d() //second ball
            ), new TrajectoryConfig(3.0, 1.0)
        );

        public static Trajectory toThird = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(), //end of toSecond
                new Pose2d() //third ball
            ), new TrajectoryConfig(3.0, 1.0)
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

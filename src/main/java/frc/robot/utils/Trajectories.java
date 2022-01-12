package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants;
import java.util.List;

public final class Trajectories {

  public static class trench6M {

    // public static final Pose2d POSE1 = new Pose2d(3.95, 3.145003 * 1.79571303587, new
    // Rotation2d());
    public static final Pose2d POSE1 = new Pose2d(3.95, 5.6475228849, new Rotation2d());

    public static final Trajectory DOWN_TRENCH =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                POSE1,
                new Pose2d(5.576245, 7.3, new Rotation2d()),
                new Pose2d(7.713587, 7.5, new Rotation2d())),
            Constants.drive.auto.CONFIG);

    public static final Pose2d POSE2 = new Pose2d(7.713587, 7.5, new Rotation2d(Math.PI));

    public static final Trajectory TO_SHOOT =
        TrajectoryGenerator.generateTrajectory(
            List.of(POSE2, new Pose2d(POSE1.getTranslation(), new Rotation2d(Math.PI))),
            Constants.drive.auto.CONFIG);
  }

  public static class trench8M {

    private static final double xAdj = 0.225;
    private static final double yAdj = 0.1;

    public static final Pose2d POSE1 = new Pose2d(3.95, 3.145003 * 1.79571303587, new Rotation2d());

    public static final Trajectory DOWN_TRENCH =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                POSE1,
                new Pose2d(3.262223 * 1.74759405074, 3.999829 * 1.79571303587, new Rotation2d()),
                new Pose2d(
                    4.549337 * 1.74759405074 + xAdj + 0.175,
                    3.999829 * 1.79571303587 + yAdj,
                    new Rotation2d())),
            Constants.drive.auto.CONFIG);

    public static final Pose2d POSE2 =
        new Pose2d(
            4.549337 * 1.74759405074 + xAdj + 0.175,
            3.999829 * 1.79571303587 + yAdj,
            new Rotation2d());

    public static final Trajectory BALL1 =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                POSE2,
                new Pose2d(
                    4.980190 * 1.74759405074 + xAdj,
                    3.958766 * 1.79571303587 + yAdj,
                    new Rotation2d())),
            Constants.drive.auto.CONFIG);

    public static final Pose2d POSE3 =
        new Pose2d(
            4.987459 * 1.74759405074 + xAdj,
            3.950242 * 1.79571303587 + yAdj,
            new Rotation2d(Math.PI / 2.0));

    public static final Trajectory BALL2 =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                POSE3,
                new Pose2d(
                    5.002947 * 1.74759405074 + xAdj,
                    4.044807 * 1.79571303587 + yAdj,
                    new Rotation2d(Math.PI / 2.0))),
            Constants.drive.auto.CONFIG);

    public static final Pose2d POSE4 =
        new Pose2d(
            4.980190 * 1.74759405074 + xAdj,
            4.045270 * 1.79571303587 + yAdj,
            new Rotation2d(Math.PI));

    public static final Trajectory TO_SHOOT =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                POSE4,
                new Pose2d(
                    3.262223 * 1.74759405074 + xAdj,
                    3.999829 * 1.79571303587 + yAdj,
                    new Rotation2d(Math.PI)),
                new Pose2d(POSE1.getTranslation(), new Rotation2d(Math.PI))),
            Constants.drive.auto.CONFIG);
  }

  public static class trench6L {

    public static final Pose2d POSE1 = new Pose2d(3.95, 7.3, new Rotation2d());

    public static final Trajectory DOWN_TRENCH =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                POSE1,
                new Pose2d(5.576245, 7.3, new Rotation2d()),
                new Pose2d(7.713587, 7.5, new Rotation2d())),
            Constants.drive.auto.CONFIG);

    public static final Pose2d POSE2 = new Pose2d(7.713587, 7.5, new Rotation2d(Math.PI));

    public static final Trajectory TO_SHOOT =
        TrajectoryGenerator.generateTrajectory(
            List.of(POSE2, new Pose2d(POSE1.getTranslation(), new Rotation2d(Math.PI))),
            Constants.drive.auto.CONFIG);
  }

  public static class trench8L {
    public static final Pose2d POSE1 = new Pose2d(3.95, 3.999829 * 1.79571303587, new Rotation2d());

    private static final double xAdj = 0.1;
    private static final double yAdj = 0.1;

    public static final Trajectory DOWN_TRENCH =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                POSE1,
                new Pose2d(3.262223 * 1.74759405074, 3.999829 * 1.79571303587, new Rotation2d()),
                new Pose2d(
                    4.549337 * 1.74759405074, 3.999829 * 1.79571303587 + xAdj, new Rotation2d())),
            Constants.drive.auto.CONFIG);

    public static final Pose2d POSE2 =
        new Pose2d(4.549337 * 1.74759405074, 3.999829 * 1.79571303587 + xAdj, new Rotation2d());

    public static final Trajectory BALL1 =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                POSE2,
                new Pose2d(
                    4.980190 * 1.74759405074 + yAdj,
                    3.958766 * 1.79571303587 + xAdj,
                    new Rotation2d())),
            Constants.drive.auto.CONFIG);

    public static final Pose2d POSE3 =
        new Pose2d(
            4.987459 * 1.74759405074 + yAdj,
            3.950242 * 1.79571303587 + xAdj,
            new Rotation2d(Math.PI / 2.0));

    public static final Trajectory BALL2 =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                POSE3,
                new Pose2d(
                    5.002947 * 1.74759405074 + yAdj,
                    4.044807 * 1.79571303587 + xAdj,
                    new Rotation2d(Math.PI / 2.0))),
            Constants.drive.auto.CONFIG);

    public static final Pose2d POSE4 =
        new Pose2d(
            4.980190 * 1.74759405074 + yAdj,
            4.045270 * 1.79571303587 + xAdj,
            new Rotation2d(Math.PI));

    public static final Trajectory TO_SHOOT =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                POSE4,
                new Pose2d(
                    3.262223 * 1.74759405074, 3.999829 * 1.79571303587, new Rotation2d(Math.PI)),
                new Pose2d(POSE1.getTranslation(), new Rotation2d(Math.PI))),
            Constants.drive.auto.CONFIG);
  }

  public static final class WheelTenBall {
    public static final Pose2d POSE1 = new Pose2d(3.95, 0.541844 * 1.79571303587, new Rotation2d());

    public static final Trajectory TO_WHEEL =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                POSE1,
                new Pose2d(3.390185 * 1.74759405074, 0.568928 * 1.79571303587, new Rotation2d())),
            Constants.drive.auto.CONFIG);

    public static final Pose2d POSE2 =
        new Pose2d(
            3.390185 * 1.74759405074, 0.568928 * 1.79571303587, new Rotation2d(Math.PI / 2.0));

    public static final Trajectory FIRST_SHOT =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                POSE2,
                new Pose2d(
                    3.190836 * 1.74759405074,
                    1.257456 * 1.79571303587,
                    new Rotation2d(Math.PI / 2.0)),
                new Pose2d(
                    3.184118 * 1.74759405074,
                    2.051534 * 1.79571303587,
                    new Rotation2d(Math.PI / 2.0))),
            Constants.drive.auto.CONFIG);

    public static final Pose2d POSE3 =
        new Pose2d(
            3.184118 * 1.74759405074, 2.051534 * 1.79571303587, Rotation2d.fromDegrees(21.923073));

    public static final Trajectory TURN_IN =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                POSE3,
                new Pose2d(
                    4.432168 * 1.74759405074,
                    2.504933 * 1.79571303587,
                    Rotation2d.fromDegrees(23.617190)),
                new Pose2d(
                    4.615875 * 1.74759405074,
                    2.792169 * 1.79571303587,
                    Rotation2d.fromDegrees(111.711319)),
                new Pose2d(
                    4.308327 * 1.74759405074,
                    2.926100 * 1.79571303587,
                    Rotation2d.fromDegrees(-153.523966)),
                new Pose2d(
                    3.881773 * 1.74759405074,
                    2.718925 * 1.79571303587,
                    Rotation2d.fromDegrees(-154.582789))),
            Constants.drive.auto.CONFIG);

    public static final Pose2d POSE4 =
        new Pose2d(
            3.881773 * 1.74759405074, 2.718925 * 1.79571303587, Rotation2d.fromDegrees(60.531868));

    public static final Trajectory SECOND_SHOOT =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                POSE4,
                new Pose2d(
                    3.941514 * 1.74759405074,
                    2.987104 * 1.79571303587,
                    Rotation2d.fromDegrees(111.213839)),
                new Pose2d(
                    3.377689 * 1.74759405074, 3.260448 * 1.79571303587, new Rotation2d(Math.PI)),
                new Pose2d(
                    3.123596 * 1.74759405074, 3.260448 * 1.79571303587, new Rotation2d(Math.PI))),
            Constants.drive.auto.CONFIG);
  }

  public static class Generator {

    public static final Pose2d POSE1 =
        new Pose2d(2.430662 * 1.74759405074, 1.926384 * 1.79571303587, new Rotation2d());

    public static final Trajectory TURN_IN =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                POSE1,
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(23.617190)),
                new Pose2d(
                    4.615875 * 1.74759405074,
                    2.792169 * 1.79571303587,
                    Rotation2d.fromDegrees(111.711319)),
                new Pose2d(
                    4.308327 * 1.74759405074,
                    2.926100 * 1.79571303587,
                    Rotation2d.fromDegrees(-153.523966)),
                new Pose2d(
                    3.881773 * 1.74759405074,
                    2.718925 * 1.79571303587,
                    Rotation2d.fromDegrees(-154.582789))),
            Constants.drive.auto.CONFIG);

    public static final Pose2d POSE2 =
        new Pose2d(
            3.881773 * 1.74759405074, 2.718925 * 1.79571303587, Rotation2d.fromDegrees(112.677865));

    public static final Trajectory TO_SHOOT =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                POSE2,
                new Pose2d(
                    3.435023 * 1.74759405074,
                    3.240016 * 1.79571303587,
                    Rotation2d.fromDegrees(174.089629)),
                new Pose2d(
                    3.147528 * 1.74759405074, 3.240016 * 1.79571303587, new Rotation2d(Math.PI))),
            Constants.drive.auto.CONFIG);
  }
}

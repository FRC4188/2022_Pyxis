// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensors;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Swerve;

/** Add your docs here. */
public class BallDetector {
    private NetworkTable table;
    private Swerve swerve = Swerve.getInstance();

    public BallDetector(String tableName) {
        table = NetworkTableInstance.getDefault().getTable(tableName);
    }

    /**
     * format {centerx, centery, size, confidence}
     * @return closest blue ball info
     */
    public double[] getClosestBlue() {
        return table.getEntry("closest blue").getDoubleArray(new double[4]);
    }

    /**
     * format {centerx, centery, size, confidence}
     * @return closest blue ball info
     */
    public double[] getClosestRed() {
        return table.getEntry("closest red").getDoubleArray(new double[4]);
    }

    public double[] getClosestBall(String selection) {
      switch(selection) {
        case "Blue":
          return getClosestBlue();
        case "Red":
          return getClosestRed();
        case "All":
          return (getClosestBlue()[2] > getClosestRed()[2]) ? getClosestBlue() : getClosestRed();
        default:
          return (DriverStation.getAlliance() == Alliance.Blue) ? getClosestBlue() : getClosestRed();
        }
    }

    public Trajectory generateTrajectory(double[] closestBall) {
        List<Pose2d> poses = new ArrayList<Pose2d>();

        Pose2d currentPose = swerve.getPose();
        double x = currentPose.getX() + Math.cos(closestBall[1]) * closestBall[0];
        double y = currentPose.getX() + Math.cos(closestBall[1]) * closestBall[0];
        Pose2d nextBall = new Pose2d(currentPose.getX() + x, currentPose.getY() + y, new Rotation2d(closestBall[1]));
        poses.add(currentPose);
        poses.add(nextBall);


        return TrajectoryGenerator.generateTrajectory(poses, Constants.drive.auto.CONFIG);
    }

    public static double[] toCoordinates(double[] info) {
      double angle = info[0] * (Constants.sensors.CAMERA_FOV / Constants.sensors.CAMERA_WIDTH);
      double distance = info[2] * (Constants.sensors.DISTANCE_SCALAR);
  
      if (info[3] > 0.7) {
        return new double[]{angle, distance};
      } else {
        return new double[]{0.0, 0.0};
      }
    }
}

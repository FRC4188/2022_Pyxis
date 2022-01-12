package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.math.Integral;

/** Class to track field position using distance traveled and heading. */
public class Odometry {
  private static Odometry instance = null;

  public static synchronized Odometry getInstance() {
    if (instance == null) instance = new Odometry();
    return instance;
  }

  // private SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.drive.KINEMATICS, new
  // Rotation2d());

  private Integral xPos = new Integral(0.0);
  private Integral yPos = new Integral(0.0);
  private Rotation2d heading = new Rotation2d();

  /**
   * Constructs a new CSPOdometry object.
   *
   * @param start The initial position of the robot.
   */
  public Odometry() {}

  /**
   * Update the odometry's estimated position.
   *
   * @param gyro Rotation of the robot as recorded by the gyro.
   * @param speeds ChassisSpeeds of the robot.
   */
  public void update(Rotation2d gyro, ChassisSpeeds speeds) {

    xPos.sample(speeds.vxMetersPerSecond);
    yPos.sample(speeds.vyMetersPerSecond);
    heading = gyro;

    SmartDashboard.putString("Odometry", getPose().toString());
    SmartDashboard.putNumber("Rotation", getPose().getRotation().getDegrees());
  }

  /** Returns the integrated position of the robot and its heading. */
  public Pose2d getPose() {
    return new Pose2d(xPos.getValue(), yPos.getValue(), heading);
  }

  /** Set a new position for the robot. */
  public void setPose(Pose2d pose) {
    xPos = new Integral(pose.getX());
    yPos = new Integral(pose.getY());
  }
}

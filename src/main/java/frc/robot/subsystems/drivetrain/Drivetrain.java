// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.sensors.Sensors;

public class Drivetrain extends SubsystemBase {
  private static Drivetrain instance;
  public static synchronized Drivetrain getInstance() {
    if (instance == null) instance = new Drivetrain();
    return instance;
  }

  private Sensors sensors = Sensors.getInstance();

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Constants.Drivetrain.Locations.FL_LOCATION, Constants.Drivetrain.Locations.FR_LOCATION, Constants.Drivetrain.Locations.BL_LOCATION, Constants.Drivetrain.Locations.BR_LOCATION);

  private SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(sensors.getHeading(), new Pose2d(), kinematics, VecBuilder.fill(0.1, 0.1, 0.05), VecBuilder.fill(0.01), VecBuilder.fill(0.35, 0.35, 0.001));

  private SlewRateLimiter xLimiter = new SlewRateLimiter(1.0);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(1.0);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(0.5);

  private SwerveModule frModule = Constants.Drivetrain.Components.FR_MODULE;
  private SwerveModule flModule = Constants.Drivetrain.Components.FL_MODULE;
  private SwerveModule brModule = Constants.Drivetrain.Components.BR_MODULE;
  private SwerveModule blModule = Constants.Drivetrain.Components.BL_MODULE;


  /** Creates a new Swerve. */
  private Drivetrain() {
    resetPose(new Pose2d(0.0, 0.0, new Rotation2d()), sensors.getHeading());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
  }

  public void updateOdometry() {
    odometry.update(sensors.getHeading(), getModuleStates());

    if (sensors.hasTarget()) {
      odometry.addVisionMeasurement(sensors.getEstimatedPose(), 0.02);
    }
  }

  public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldOriented) {
    xSpeed = -xLimiter.calculate(xSpeed) * Constants.Drivetrain.MAX_SPEED;
    ySpeed = yLimiter.calculate(ySpeed) * Constants.Drivetrain.MAX_SPEED;
    rotSpeed = rotLimiter.calculate(rotSpeed) * Constants.Drivetrain.MAX_ROT_SPEED;

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(fieldOriented ? 
      ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, sensors.getHeading()) :
      new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));

    setModuleStates(states);
  }

  public void disable() {
    frModule.disable();
    flModule.disable();
    brModule.disable();
    blModule.disable();
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drivetrain.MAX_SPEED);

    frModule.setState(states[0]);
    flModule.setState(states[1]);
    brModule.setState(states[2]);
    blModule.setState(states[3]);
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
      setModuleStates(kinematics.toSwerveModuleStates(new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond)));
  }

  public void resetPose(Pose2d pose, Rotation2d rotation) {
    odometry.resetPosition(pose, rotation);
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[]{flModule.getState(), frModule.getState(), blModule.getState(), brModule.getState()};
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  public double getSpeed() {
    ChassisSpeeds speeds = getChassisSpeeds();
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);  
  }
}

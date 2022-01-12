// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.sensors.Sensors;

public class Swerve extends SubsystemBase {

  private static Swerve instance = null;

  public static synchronized Swerve getInstance() {
    if (instance == null) instance = new Swerve();

    return instance;
  }

  private Module leftFront = new Module(1, 2, 21, Constants.drive.M1_ZERO);
  private Module rightFront = new Module(3, 4, 22, Constants.drive.M2_ZERO);
  private Module leftRear = new Module(5, 6, 23, Constants.drive.M3_ZERO);
  private Module rightRear = new Module(7, 8, 24, Constants.drive.M4_ZERO);

  private Odometry odometry = Odometry.getInstance();

  private final Field2d field = new Field2d();

  private Sensors sensors = Sensors.getInstance();

  // private Kinematics kinematics = Constants.drive.KINEMATICS;
  SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          Constants.drive.FrontLeftLocation,
          Constants.drive.FrontRightLocation,
          Constants.drive.BackLeftLocation,
          Constants.drive.BackRightLocation);

  private PIDController rotationPID = new PIDController(0.1, 0.0, 0.01);

  private Notifier dashboard = new Notifier(() -> smartDashboard());
  private Notifier odometryNotifier = new Notifier(() -> runOdo());

  /** Creates a new Swerve. */
  private Swerve() {
    CommandScheduler.getInstance().registerSubsystem(this);
    rotationPID.enableContinuousInput(-180, 180);
    rotationPID.setTolerance(2.0);

    SmartDashboard.putData("Field", field);

    if (Robot.isReal()) dashboard.startPeriodic(0.2);
    else dashboard.startPeriodic(0.05);
    odometryNotifier.startPeriodic(0.05);
  }

  @Override
  public void periodic() {}

  private void runOdo() {
    odometry.update(sensors.getRotation(), getChassisSpeeds());
    field.setRobotPose(odometry.getPose());
  }

  public void stop() {
    leftFront.stop();
    rightFront.stop();
    leftRear.stop();
    rightRear.stop();
  }

  private void smartDashboard() {
    SmartDashboard.putNumber("LF Angle", leftFront.getAbsoluteAngle());
    SmartDashboard.putNumber("RF Angle", rightFront.getAbsoluteAngle());
    SmartDashboard.putNumber("LR Angle", leftRear.getAbsoluteAngle());
    SmartDashboard.putNumber("RR Angle", rightRear.getAbsoluteAngle());
  }

  public void drive(double yInput, double xInput, double rotInput, boolean fieldOriented) {
    yInput *= -Constants.drive.MAX_VELOCITY;
    xInput *= Constants.drive.MAX_VELOCITY;
    rotInput *= 4.0 * Math.PI;

    if (rotInput != 0.0) {
      setRotSetpoint(-sensors.getRotation().getDegrees());
    } else {
      if (yInput != 0 || xInput != 0) {
        double correction = rotationPID.calculate(-sensors.getRotation().getDegrees());
        rotInput = rotationPID.atSetpoint() ? 0.0 : correction;
      }
    }

    setChassisSpeeds(
        !fieldOriented
            ? ChassisSpeeds.fromFieldRelativeSpeeds(yInput, xInput, rotInput, sensors.getRotation())
            : new ChassisSpeeds(yInput, xInput, rotInput));
  }

  public void setRotSetpoint(double setpoint) {
    rotationPID.setSetpoint(setpoint);
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    if (Robot.isSimulation()) odometry.update(new Rotation2d(), speeds);
    else setModuleStates(kinematics.toSwerveModuleStates(speeds));
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.drive.MAX_VELOCITY);

    leftFront.setModuleState(
        new SwerveModuleState(states[0].speedMetersPerSecond, states[0].angle));
    rightFront.setModuleState(
        new SwerveModuleState(states[1].speedMetersPerSecond, states[1].angle));
    leftRear.setModuleState(new SwerveModuleState(states[2].speedMetersPerSecond, states[2].angle));
    rightRear.setModuleState(
        new SwerveModuleState(states[3].speedMetersPerSecond, states[3].angle));
  }

  public ChassisSpeeds getChassisSpeeds() {
    ChassisSpeeds temp = kinematics.toChassisSpeeds(getModuleStates());

    return new ChassisSpeeds(
        -temp.vxMetersPerSecond, -temp.vyMetersPerSecond, temp.omegaRadiansPerSecond);
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      leftFront.getModuleState(),
      rightFront.getModuleState(),
      leftRear.getModuleState(),
      rightRear.getModuleState()
    };
  }

  public Pose2d getPose() {
    return odometry.getPose();
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public double getVelocity() {
    return Math.sqrt(Math.pow(getChassisSpeeds().vxMetersPerSecond, 2) + Math.pow(getChassisSpeeds().vyMetersPerSecond, 2));
  }

  public double getFrontLeftAngleTemp() {
    return leftFront.getAngleTemp();
  }

  public double getFrontRightAngleTemp() {
    return rightFront.getAngleTemp();
  }

  public double getRearLeftAngleTemp() {
    return leftRear.getAngleTemp();
  }

  public double getRearRightAngleTemp() {
    return rightRear.getAngleTemp();
  }

  public double getFrontLeftDriveTemp() {
    return leftFront.getSpeedTemp();
  }

  public double getFrontRightDriveTemp() {
    return rightFront.getSpeedTemp();
  }

  public double getRearLeftDriveTemp() {
    return leftRear.getSpeedTemp();
  }

  public double getRearRightDriveTemp() {
    return rightRear.getSpeedTemp();
  }
}

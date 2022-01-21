// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Swerve;

public class FollowTrajectory extends CommandBase {

  Trajectory trajectory;
  Rotation2d rotation;

  Swerve swerve = Swerve.getInstance();
  //Odometry odometry = Odometry.getInstance();

  private static HolonomicDriveController controller;
  private static Timer timer = new Timer();

  /** Creates a new FollowTrajectory. */
  public FollowTrajectory(Trajectory trajectory, Rotation2d rotation) {
    addRequirements(swerve);

    this.trajectory = trajectory;
    this.rotation = rotation;

    ProfiledPIDController thetaController = Constants.drive.thetaPID;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    controller = new HolonomicDriveController(Constants.drive.xPID, Constants.drive.yPID, thetaController);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.setChassisSpeeds(controller.calculate(swerve.getPose(), trajectory.sample(timer.get()), rotation));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.setChassisSpeeds(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > trajectory.getTotalTimeSeconds();
  }
}

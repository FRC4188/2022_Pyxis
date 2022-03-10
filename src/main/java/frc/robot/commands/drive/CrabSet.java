// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CrabSet extends CommandBase {

  Swerve swerve = Swerve.getInstance();

  double velocity, angle;

  public CrabSet(double velocity, double angle) {
    addRequirements(swerve);
    this.velocity = velocity;
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {
    swerve.setModuleStates(new SwerveModuleState[] {
      new SwerveModuleState(velocity, Rotation2d.fromDegrees(angle)),
      new SwerveModuleState(velocity, Rotation2d.fromDegrees(angle)),
      new SwerveModuleState(velocity, Rotation2d.fromDegrees(angle)),
      new SwerveModuleState(velocity, Rotation2d.fromDegrees(angle))
    });
  }

  @Override
  public void end(boolean interrupted) {
    velocity = 0.0;
    angle = 0.0;

    swerve.setModuleStates(new SwerveModuleState[] {
      new SwerveModuleState(velocity, Rotation2d.fromDegrees(angle)),
      new SwerveModuleState(velocity, Rotation2d.fromDegrees(angle)),
      new SwerveModuleState(velocity, Rotation2d.fromDegrees(angle)),
      new SwerveModuleState(velocity, Rotation2d.fromDegrees(angle))
    });
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

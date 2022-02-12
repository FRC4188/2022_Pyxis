// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sensors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.sensors.Sensors;

public class ResetRotation extends InstantCommand {

  private Rotation2d rotation;

  public ResetRotation() {
    this(new Rotation2d());
  }

  public ResetRotation(Rotation2d rotation) {
    this.rotation = rotation;

    addRequirements(Swerve.getInstance());
  }

  @Override
  public void initialize() {
    Pose2d pose = Swerve.getInstance().getPose();
    Swerve.getInstance().setPose(new Pose2d(pose.getTranslation(), rotation));
    Swerve.getInstance().setRotSetpoint(0.0);
  }
}

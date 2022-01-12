// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sensors;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.sensors.Sensors;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetGyro extends InstantCommand {

  Sensors sensors = Sensors.getInstance();
  Rotation2d angle;

  public ResetGyro(Rotation2d angle) {
    this.angle = angle;
  }

  public ResetGyro() {
    this(new Rotation2d());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sensors.setPigeonAngle(angle.getDegrees());
    Swerve.getInstance().setRotSetpoint(0.0);
  }
}

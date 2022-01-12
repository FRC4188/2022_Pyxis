// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sensors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Odometry;
import frc.robot.subsystems.sensors.Sensors;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetOdometry extends InstantCommand {

  Sensors sensors = Sensors.getInstance();
  Odometry odometry = Odometry.getInstance();
  Pose2d pose = null;

  public ResetOdometry(Pose2d pose) {
    this.pose = pose;
  }

  public ResetOdometry() {
    this(new Pose2d());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    odometry.setPose(pose);
    sensors.setPigeonAngle(pose.getRotation().getDegrees());
  }
}

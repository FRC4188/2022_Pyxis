// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sensors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.sensors.Sensors;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetPose extends InstantCommand {

  Pose2d newPose;

  public ResetPose() {
    newPose = new Pose2d();
  }

  public ResetPose(Pose2d newPose) {
    this.newPose = newPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Swerve.getInstance().setPose(newPose);
    Sensors.getInstance().setRotation(newPose.getRotation());
  }
}

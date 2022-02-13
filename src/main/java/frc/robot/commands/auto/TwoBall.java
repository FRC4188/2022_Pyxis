// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.commands.groups.AutoShoot;
import frc.robot.commands.sensors.ResetPose;
import frc.robot.commands.sensors.ResetRotation;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.utils.paths.Trajectories;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBall extends SequentialCommandGroup {

  Swerve drive = Swerve.getInstance();

  /** Creates a new TwoBall. */
  public TwoBall() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetRotation(),
      new ResetPose(),
      new FollowTrajectory(Trajectories.twoball.first, Rotation2d.fromDegrees(24.13)),
      new AutoShoot().withTimeout(10.0),
      new FollowTrajectory(Trajectories.twoball.second, Rotation2d.fromDegrees(-203.03))
    );
  }
}

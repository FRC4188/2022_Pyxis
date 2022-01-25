// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.utils.paths.Trajectories;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBall extends SequentialCommandGroup {
  /** Creates a new FiveBall. */
  public FiveBall() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new FollowTrajectory(Trajectories.fiveball.first, Rotation2d.fromDegrees(24.13)),
      new FollowTrajectory(Trajectories.fiveball.second, Rotation2d.fromDegrees(-48.03)),
      new WaitCommand(3.0),
      new FollowTrajectory(Trajectories.fiveball.terminal, Rotation2d.fromDegrees(-22.98)),
      new WaitCommand(5.0),
      new FollowTrajectory(Trajectories.fiveball.lastShoot, Rotation2d.fromDegrees(136.45))
    );
  }
}

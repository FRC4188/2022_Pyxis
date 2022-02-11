// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import org.ejml.ops.FOperatorBinary;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.commands.shooter.SpinShooter;
import frc.robot.commands.turret.SetToAngle;
import frc.robot.commands.turret.TrackTarget;
import frc.robot.utils.paths.Trajectories;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneBall extends SequentialCommandGroup {
  /** Creates a new TwoBall. */
  public OneBall() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(
        new SetToAngle(180), 
        new SpinShooter(() -> 3000)),
      new TrackTarget(true),
      new FollowTrajectory(Trajectories.oneball.first, Rotation2d.fromDegrees(-37.4)),
      new TrackTarget(false)
    );
  }
}

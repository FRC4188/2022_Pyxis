// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.commands.sensors.ResetPose;
import java.util.List;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestOTFRadial extends SequentialCommandGroup {
  /** Creates a new TestOTFRadial. */
  public TestOTFRadial() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ResetPose(),
        new FollowTrajectory(
            TrajectoryGenerator.generateTrajectory(
                List.of(
                    new Pose2d(0.0, 0.0, new Rotation2d()), new Pose2d(5.0, 0.0, new Rotation2d())),
                new TrajectoryConfig(1.5, 3.0)),
            new Rotation2d()));
  }
}

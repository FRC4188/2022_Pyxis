// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.commands.sensors.ResetPose;
import frc.robot.planning.Trajectories;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuto extends SequentialCommandGroup {
  /** Creates a new TestAuto. */
  public TestAuto() {
    addCommands(
      new ResetPose(),
      new RunCommand(() -> {}).withTimeout(5.0),
      //new FollowTrajectory(Trajectories.Auto1.line, new Rotation2d(Math.PI))
      new FollowTrajectory(Trajectories.middle.down, Trajectories.middle.headings[0]),
      new FollowTrajectory(Trajectories.middle.ball1, Trajectories.middle.headings[1]),
      new FollowTrajectory(Trajectories.middle.ball2, Trajectories.middle.headings[2]),
      new FollowTrajectory(Trajectories.middle.shoot1, Trajectories.middle.headings[3]),
      new FollowTrajectory(Trajectories.middle.mid1, Trajectories.middle.headings[4]),
      new FollowTrajectory(Trajectories.middle.mid2, Trajectories.middle.headings[4]),
      new FollowTrajectory(Trajectories.middle.shoot2, Trajectories.middle.headings[5])
    );
  }
}

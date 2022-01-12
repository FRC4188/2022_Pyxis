// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.commands.sensors.ResetOdometry;
import frc.robot.utils.Trajectories;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Steal10 extends SequentialCommandGroup {
  /** Creates a new Steal10. */
  public Steal10() {
    if (Robot.isReal()) {
      addCommands();
    } else {
      addCommands(
          new ResetOdometry(Trajectories.WheelTenBall.POSE1),
          new FollowTrajectory(Trajectories.WheelTenBall.TO_WHEEL, new Rotation2d()),
          new FollowTrajectory(Trajectories.WheelTenBall.FIRST_SHOT, new Rotation2d()),
          new FollowTrajectory(Trajectories.WheelTenBall.TURN_IN, new Rotation2d()),
          new FollowTrajectory(Trajectories.WheelTenBall.SECOND_SHOOT, new Rotation2d()));
    }
  }
}

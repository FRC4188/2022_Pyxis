package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.commands.groups.AutoShoot;
import frc.robot.commands.sensors.ResetPose;
import frc.robot.commands.sensors.ResetRotation;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.utils.paths.Trajectories;

public class TwoBall extends SequentialCommandGroup {

  Swerve drive = Swerve.getInstance();

  /** Creates a new TwoBall. */
  public TwoBall() {
    addCommands(
      new ResetRotation(),
      new ResetPose(),
      new FollowTrajectory(Trajectories.twoball.first, Rotation2d.fromDegrees(24.13)),
      new AutoShoot().withTimeout(10.0),
      new FollowTrajectory(Trajectories.twoball.second, Rotation2d.fromDegrees(-203.03))
    );
  }
}

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.utils.paths.Trajectories;

public class ThreeBall extends SequentialCommandGroup {
  /** Creates a new ThreeBall. */
  public ThreeBall() {
    addCommands(
      new FollowTrajectory(Trajectories.threeball.toFirst, Rotation2d.fromDegrees(-32.4)),
      new WaitCommand(2.0),
      new FollowTrajectory(Trajectories.threeball.toSecond, Rotation2d.fromDegrees(157.37)),
      new FollowTrajectory(Trajectories.threeball.toThird, Rotation2d.fromDegrees(147.08))
    );
  }
}

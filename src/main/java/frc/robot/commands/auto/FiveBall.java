package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.utils.paths.Trajectories;

public class FiveBall extends SequentialCommandGroup {
  /**
   * <p>
   * Creates a new FiveBall command.
   * </p>
   * <p>
   * Shoots first ball, collects two from field, shoots those, collects two from terminal, shoots those.
   * </p>
   */
  public FiveBall() {
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

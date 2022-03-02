package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.commands.groups.AutoIntake;
import frc.robot.commands.trigger.AutoFireQuantity;
import frc.robot.utils.paths.Trajectories;

public class FiveBall extends SequentialCommandGroup {
  /**
   * Creates a new FiveBall command.
   * <p>
   * Shoots first ball, collects two from field, shoots those, collects two from terminal, shoots those.
   */
  public FiveBall() {
    addCommands(
      new ParallelDeadlineGroup(
        new FollowTrajectory(Trajectories.fiveball.first, Rotation2d.fromDegrees(0.0)),
        new AutoIntake()
      ),
      new AutoFireQuantity(2),
      new ParallelDeadlineGroup(
        new FollowTrajectory(Trajectories.fiveball.second, Rotation2d.fromDegrees(-59.45)),
        new AutoIntake()
      ),
      new AutoFireQuantity(1)
    );
  }
}

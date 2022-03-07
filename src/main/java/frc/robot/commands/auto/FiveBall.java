package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.commands.groups.AutoIntake;
import frc.robot.commands.groups.AutoShootQuantity;
import frc.robot.commands.shooter.HoodAngle;
import frc.robot.commands.shooter.ShooterVelocity;
import frc.robot.commands.turret.SetToAngle;
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
        new AutoIntake(),
        new HoodAngle(()->21.0),
        new ShooterVelocity(()->2730.0)
      ),
      new AutoShootQuantity(2, true),
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new FollowTrajectory(Trajectories.fiveball.terminal1, Rotation2d.fromDegrees(-50.0)),
          new FollowTrajectory(Trajectories.fiveball.terminal2, Rotation2d.fromDegrees(-50.0)),
          new FollowTrajectory(Trajectories.fiveball.shoot2, Rotation2d.fromDegrees(119.9))
        ),
        new WaitCommand(1.0).andThen(new AutoIntake()),
        new SetToAngle(-180.0),
        new HoodAngle(()->30.0),
        new ShooterVelocity(()->3180.0)
      ),
      new AutoShootQuantity(1, true),
      new FollowTrajectory(Trajectories.fiveball.shoot3, Rotation2d.fromDegrees(116.0)),
      new AutoShootQuantity(2, true)
    );
  }
}

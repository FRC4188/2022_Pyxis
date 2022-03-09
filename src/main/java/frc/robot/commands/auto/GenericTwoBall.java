package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.commands.groups.AutoIntake;
import frc.robot.commands.groups.AutoShootQuantity;
import frc.robot.commands.groups.PresetShootQuantity;
import frc.robot.commands.sensors.ResetPose;
import frc.robot.commands.sensors.ResetRotation;
import frc.robot.commands.shooter.HoodAngle;
import frc.robot.commands.shooter.ShooterVelocity;
import frc.robot.commands.turret.SetToAngle;
import frc.robot.utils.paths.Trajectories;

public class GenericTwoBall extends SequentialCommandGroup {
  /**
   * Creates a new FiveBall command.
   * <p>
   * Shoots first ball, collects two from field, shoots those, collects two from terminal, shoots those.
   */
  public GenericTwoBall() {
    addCommands(
      new ResetPose(),
      new ResetRotation(),
      new ParallelDeadlineGroup(
        new FollowTrajectory(Trajectories.twoball.toFirst, Rotation2d.fromDegrees(0.0)),
        new AutoIntake(),
        new HoodAngle(()->21.0),
        new ShooterVelocity(()->2730.0)
      ),
      new PresetShootQuantity(22.8, 2650.0, 2, true)
    );
  }
}
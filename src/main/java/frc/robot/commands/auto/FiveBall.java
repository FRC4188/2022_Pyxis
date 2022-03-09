package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.commands.groups.AutoIntake;
import frc.robot.commands.groups.PresetShootQuantity;
import frc.robot.commands.sensors.ResetPose;
import frc.robot.commands.sensors.ResetRotation;
import frc.robot.commands.shooter.HoodAngle;
import frc.robot.commands.shooter.ShooterVelocity;
import frc.robot.commands.turret.SetToAngle;
import frc.robot.subsystems.drive.Trajectories;

public class FiveBall extends SequentialCommandGroup {
  /**
   * Creates a new FiveBall command.
   * <p>
   * Shoots first ball, collects two from field, shoots those, collects two from terminal, shoots those.
   */
  public FiveBall() {
    addCommands(
      new ResetPose(),
      new ResetRotation(),
      new ParallelDeadlineGroup(
        new FollowTrajectory(Trajectories.fiveball.first, Rotation2d.fromDegrees(0.0)),
        new AutoIntake(),
        new HoodAngle(()->21.0),
        new ShooterVelocity(()->2730.0)
      ),
      new PresetShootQuantity(22.8, 2650.0, 2, true).withTimeout(2.0),
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new FollowTrajectory(Trajectories.fiveball.terminal1, Rotation2d.fromDegrees(-50.0)),
          new FollowTrajectory(Trajectories.fiveball.terminal2, Rotation2d.fromDegrees(-50.0)),
          new FollowTrajectory(Trajectories.fiveball.shoot2, Rotation2d.fromDegrees(119.9))
        ),
        new AutoIntake(),
        new SetToAngle(-180.0),
        new HoodAngle(()->30.0),
        new ShooterVelocity(()->3180.0)
      ),
      new PresetShootQuantity(37.5, 3050.0, 1, true).withTimeout(1.5),
      new FollowTrajectory(Trajectories.fiveball.shoot3, Rotation2d.fromDegrees(116.0)),
      new PresetShootQuantity(34.5, 3050.0, 2, true)
    );
  }
}

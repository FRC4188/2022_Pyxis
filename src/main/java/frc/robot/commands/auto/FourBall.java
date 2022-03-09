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

public class FourBall extends SequentialCommandGroup {
  /**
   * Creates a new FiveBall command.
   * <p>
   * Shoots first ball, collects two from field, shoots those, collects two from terminal, shoots those.
   */
  public FourBall() {
    addCommands(
      new ResetPose(),
      new ResetRotation(),
      new ParallelDeadlineGroup(
        new FollowTrajectory(Trajectories.fourball.toFirst, Rotation2d.fromDegrees(0.0)),
        new AutoIntake(),
        new HoodAngle(()->21.0),
        new ShooterVelocity(()->2730.0)
      ),
      new PresetShootQuantity(22.8, 2650.0, 2, true).withTimeout(2.5),
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new FollowTrajectory(Trajectories.fourball.toSecond, Rotation2d.fromDegrees(89.37)),
          new FollowTrajectory(Trajectories.fourball.toThird, Rotation2d.fromDegrees(-168.62))
        ),
        new AutoIntake(),
        new SetToAngle(-245.0),
        new HoodAngle(()->30.0),
        new ShooterVelocity(()->3180.0)
      ),
      new PresetShootQuantity(34.5, 3050.0, 2, true)
    );
  }
}

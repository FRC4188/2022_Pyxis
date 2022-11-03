package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.commands.groups.PresetShootQuantity;
import frc.robot.commands.indexer.LoadBalls;
import frc.robot.commands.intake.SpinIntake;
import frc.robot.commands.sensors.ResetPose;
import frc.robot.commands.shooter.HoodAngle;
import frc.robot.commands.shooter.ShooterVelocity;
import frc.robot.commands.trigger.PushTrigger;
import frc.robot.subsystems.drive.Trajectories;

public class GenericTwoBall extends SequentialCommandGroup {
  /**
   * Creates a new FiveBall command.
   *
   * <p>Shoots first ball, collects two from field, shoots those, collects two from terminal, shoots
   * those.
   */
  public GenericTwoBall() {
    addCommands(
        new ResetPose(new Pose2d(0.0, 0.0, new Rotation2d(2.37))),
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new FollowTrajectory(Trajectories.hoard.first, new Rotation2d(2.37))),
            new SpinIntake(8.5),
            new LoadBalls(),
            new ShooterVelocity(() -> 0.0),
            new HoodAngle(() -> 0.0),
            new PushTrigger(0.0)),
        new PresetShootQuantity(23.0, 2700.0, 2, true));
  }
}

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.commands.groups.AutoIntake;
import frc.robot.commands.shooter.HoodAngle;
import frc.robot.commands.shooter.ShooterVoltage;
import frc.robot.commands.trigger.PushTrigger;
import frc.robot.commands.turret.TrackTarget;
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
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new FollowTrajectory(Trajectories.fiveball.first, Rotation2d.fromDegrees(0.0)),
          new ParallelDeadlineGroup(
            new SequentialCommandGroup(
              new WaitCommand(2.0),
              new PushTrigger(8.0).withTimeout(1.0)
            ),
            new TrackTarget(true),
    
            new HoodAngle(13.0),
            new ShooterVoltage(4.7)
          ),
          new FollowTrajectory(Trajectories.fiveball.second, Rotation2d.fromDegrees(-59.45))
        ),
        new AutoIntake()
      ),
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new WaitCommand(2.0),
          new PushTrigger(8.0).withTimeout(1.0)
        ),
        new TrackTarget(true),
        new AutoIntake(),
        new HoodAngle(13.0),
        new ShooterVoltage(4.7)
      )
    );
  }
}

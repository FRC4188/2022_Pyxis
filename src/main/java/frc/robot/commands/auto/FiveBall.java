package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.commands.groups.AutoIntake;
import frc.robot.commands.groups.AutoShoot;
import frc.robot.commands.groups.AutoShootQuantity;
import frc.robot.commands.groups.PresetShootQuantity;
import frc.robot.commands.intake.SpinIntake;
import frc.robot.commands.sensors.ResetPose;
import frc.robot.commands.sensors.ResetRotation;
import frc.robot.commands.shooter.FindHoodZeros;
import frc.robot.commands.shooter.HoodAngle;
import frc.robot.commands.shooter.ShooterVelocity;
import frc.robot.commands.turret.SetToAngle;
import frc.robot.subsystems.drive.Trajectories;
import frc.robot.subsystems.intake.Intake;
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
        new FollowTrajectory(Trajectories.twoball.toFirst, Rotation2d.fromDegrees(0.0)),
        new FindHoodZeros(),
        new AutoIntake()
      ),
      new AutoShootQuantity(2, true).withTimeout(2.5),
      new InstantCommand(() -> Intake.getInstance().raise(true)),
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new FollowTrajectory(Trajectories.fiveball.terminal1, Rotation2d.fromDegrees(-50.0)),
          new FollowTrajectory(Trajectories.fiveball.terminal2, Rotation2d.fromDegrees(-50.0)),
          new FollowTrajectory(Trajectories.fiveball.shoot2, Rotation2d.fromDegrees(119.9))
        ),
        new WaitCommand(1.0).andThen(new AutoIntake()),
        new SetToAngle(-180.0)
      ),
      new AutoShootQuantity(1, false).withTimeout(1.25),
      new ParallelDeadlineGroup(
        new FollowTrajectory(Trajectories.fiveball.shoot3, Rotation2d.fromDegrees(116.0)),
        new AutoIntake()
      ),
      new ResetRotation(Rotation2d.fromDegrees(26.0)),
      new AutoShoot(false)
    );
  }
}

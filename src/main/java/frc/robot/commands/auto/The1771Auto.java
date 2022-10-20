package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.commands.groups.PresetShoot;
import frc.robot.commands.groups.PresetShootQuantity;
import frc.robot.commands.intake.SpinIntake;
import frc.robot.commands.sensors.ResetPose;
import frc.robot.commands.sensors.ResetRotation;
import frc.robot.commands.shooter.HoodAngle;
import frc.robot.commands.shooter.ShooterVelocity;
import frc.robot.commands.trigger.PushTrigger;
import frc.robot.commands.turret.SetToAngle;
import frc.robot.commands.turret.TrackTarget;
import frc.robot.subsystems.drive.Trajectories;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.trigger.PreShooter;
public class The1771Auto extends SequentialCommandGroup {
  /**
   * Creates a new FiveBall command.
   * <p>
   * Shoots first ball, collects two from field, shoots those, collects two from terminal, shoots those.
   */
  public The1771Auto() {
    addCommands(
      new ResetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(-90.0))),
      new ResetRotation(Rotation2d.fromDegrees(-90.0)),
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new FollowTrajectory(Trajectories.fiveballplus.first, Rotation2d.fromDegrees(-90.0)),
          new FollowTrajectory(Trajectories.fiveballplus.second, Rotation2d.fromDegrees(180.0))
        ),
        new AutoIntake(),
        new SetToAngle(-30.0)
      ),
      new PresetShootQuantity(26.0, 2600.0, 2, true),
      new PresetShootQuantity(26.0, 2600.0, 1, true).withTimeout(1.25),
      new InstantCommand(() -> PreShooter.getInstance().setVoltage(0.0)),
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new FollowTrajectory(Trajectories.fiveballplus.third, new Rotation2d(-2.45)),
          new FollowTrajectory(Trajectories.sixball.fourth, new Rotation2d(-2.47)),
          new FollowTrajectory(Trajectories.sixball.fifth, new Rotation2d(0.83))
        ),
        new AutoIntake(),
        new SetToAngle(-110.0)
      ),
      new PresetShoot(34.0, 2900.0)
    );
  }
}

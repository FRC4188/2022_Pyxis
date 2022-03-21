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
import frc.robot.commands.groups.PresetShootQuantity;
import frc.robot.commands.intake.SpinIntake;
import frc.robot.commands.sensors.ResetPose;
import frc.robot.commands.sensors.ResetRotation;
import frc.robot.commands.shooter.FindHoodZeros;
import frc.robot.commands.shooter.HoodAngle;
import frc.robot.commands.shooter.ShooterVelocity;
import frc.robot.commands.trigger.PushTrigger;
import frc.robot.commands.turret.SetToAngle;
import frc.robot.commands.turret.TrackTarget;
import frc.robot.subsystems.drive.Trajectories;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.trigger.PreShooter;
public class FiveBall extends SequentialCommandGroup {
  /**
   * Creates a new FiveBall command.
   * <p>
   * Shoots first ball, collects two from field, shoots those, collects two from terminal, shoots those.
   */
  public FiveBall() {
    addCommands(
      new ResetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(-90.0))),
      new ResetRotation(Rotation2d.fromDegrees(-90.0)),
      new ParallelDeadlineGroup(
        new FollowTrajectory(Trajectories.twoball.toFirst.relativeTo(new Pose2d(0, 0, Rotation2d.fromDegrees(90.0))), Rotation2d.fromDegrees(-90.0)),
        new FindHoodZeros(),
        new AutoIntake()
      ),
      new SequentialCommandGroup(
        new PresetShootQuantity(20.0, 2500.0, 2, true).withTimeout(2.5)//20 and 2500,
      ),
      new InstantCommand(() -> {
        Intake.getInstance().raise(true);
        Intake.getInstance().setVoltage(0.0);
      }, Intake.getInstance()),
      new InstantCommand(() -> PreShooter.getInstance().setVoltage(0.0)),
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new FollowTrajectory(Trajectories.fiveball.terminal1.relativeTo(new Pose2d(0, 0, Rotation2d.fromDegrees(90.0))), Rotation2d.fromDegrees(-140.0)),
          new FollowTrajectory(Trajectories.fiveball.terminal2.relativeTo(new Pose2d(0, 0, Rotation2d.fromDegrees(90.0))), Rotation2d.fromDegrees(-140.0)),
          new FollowTrajectory(Trajectories.fiveball.shoot2.relativeTo(new Pose2d(0, 0, Rotation2d.fromDegrees(90.0))), Rotation2d.fromDegrees(54.18-90.0))
        ),
        new WaitCommand(1.0).andThen(new AutoIntake()),
        new SetToAngle(-270.0)
      ),
      new PresetShootQuantity(24.0, 2600.0, 3, true).withTimeout(2.5),//20 and 2500,
      /*new ParallelDeadlineGroup(
        new FollowTrajectory(Trajectories.fiveball.shoot3, Rotation2d.fromDegrees(53.45)),
        new AutoIntake()
      ),*/
      new AutoShoot(false)
    );
  }
}

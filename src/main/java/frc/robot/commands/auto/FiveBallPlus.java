package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.commands.intake.SpinIntake;
import frc.robot.commands.sensors.ResetPose;
import frc.robot.commands.sensors.ResetRotation;
import frc.robot.commands.shooter.HoodAngle;
import frc.robot.subsystems.drive.Trajectories;
public class FiveBallPlus extends SequentialCommandGroup {
  /**
   * Creates a new FiveBall command.
   * <p>
   * Shoots first ball, collects two from field, shoots those, collects two from terminal, shoots those.
   */
  public FiveBallPlus() {
    addCommands(
      new ResetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(-90.0))),
      new ResetRotation(Rotation2d.fromDegrees(-90.0)),
      //new InstantCommand(() -> new SetToAngle(-360.0).schedule(false)),
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new FollowTrajectory(Trajectories.fiveballplus.first, Rotation2d.fromDegrees(-90.0)),
          new FollowTrajectory(Trajectories.fiveballplus.second, Rotation2d.fromDegrees(180.0)),
          new WaitCommand(0.5),
          new FollowTrajectory(Trajectories.fiveballplus.third, new Rotation2d(-2.45)),
          new WaitCommand(1.0),
          new FollowTrajectory(Trajectories.fiveballplus.fourth, new Rotation2d(-2.45)),
          new FollowTrajectory(Trajectories.fiveballplus.fifth, new Rotation2d(Math.PI))
        ),
        //new HoodAngle(() -> 10.0).withTimeout(0.1).andThen(new FindHoodZeros()),
        new SpinIntake(8.5)
      )
      //new PresetShoot(34.0, 2900.0)
    );
  }
}

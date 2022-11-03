// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.commands.groups.AutoIntake;
import frc.robot.commands.groups.AutoShoot;
import frc.robot.commands.groups.AutoShootQuantity;
import frc.robot.commands.indexer.SpinIndexer;
import frc.robot.commands.intake.SpinIntake;
import frc.robot.commands.sensors.ResetPose;
import frc.robot.commands.trigger.PushTrigger;
import frc.robot.subsystems.drive.Trajectories;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StealAuto extends SequentialCommandGroup {
  /** Creates a new StealAuto. */
  public StealAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ResetPose(new Pose2d(6.728, 5.891, new Rotation2d(-0.911, 1.901))),
        new AutoShootQuantity(1, false),
        new ParallelDeadlineGroup(
            new FollowTrajectory(Trajectories.steal.first, new Rotation2d(-0.416, 0.787)),
            new AutoIntake()),
        new ParallelDeadlineGroup(
            new FollowTrajectory(Trajectories.steal.second, new Rotation2d(0.0, 1.669)),
            new SpinIntake(0.0)),
        new ParallelCommandGroup(
                new PushTrigger(-8.0), new SpinIndexer(-8.0), new SpinIntake(-12.0, true))
            .withTimeout(2.5),
        new AutoIntake().withTimeout(3.0),
        new AutoShoot(false));
  }
}

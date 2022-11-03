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
import frc.robot.commands.groups.AutoShootQuantity;
import frc.robot.commands.indexer.LoadBalls;
import frc.robot.commands.indexer.SpinIndexer;
import frc.robot.commands.intake.SpinIntake;
import frc.robot.commands.sensors.ResetPose;
import frc.robot.commands.shooter.HoodAngle;
import frc.robot.commands.shooter.ShooterVelocity;
import frc.robot.commands.trigger.PushTrigger;
import frc.robot.subsystems.drive.Trajectories;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HoardingAuto extends SequentialCommandGroup {
  /** Creates a new HoardingAuto. */
  public HoardingAuto() {

    // Turret turret = Turret.getInstance();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
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

        // new PresetShootQuantity(23.0, 2700.0, 2, true),
        new AutoShootQuantity(2, true),
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new FollowTrajectory(Trajectories.hoard.second, new Rotation2d(-1.59)),
                new FollowTrajectory(Trajectories.hoard.third, new Rotation2d(1.42)),
                new FollowTrajectory(Trajectories.hoard.fourth, new Rotation2d(2.66))),
            new SpinIntake(8.5),
            new LoadBalls(),
            new ShooterVelocity(() -> 2700.0),
            new HoodAngle(() -> 23.0),
            new PushTrigger(0.0)),
        new ParallelCommandGroup(
            new PushTrigger(-8.0), new SpinIndexer(-8.0), new SpinIntake(-6.0, true)));
  }
}

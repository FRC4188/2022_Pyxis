// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.commands.groups.AutoIntake;
import frc.robot.commands.groups.AutoShoot;
import frc.robot.commands.hopper.SpinHopper;
import frc.robot.commands.intake.SpinIntake;
import frc.robot.commands.sensors.ResetGyro;
import frc.robot.commands.sensors.ResetOdometry;
import frc.robot.commands.shooter.ShooterVelocity;
import frc.robot.commands.turret.TurretAngle;
import frc.robot.commands.turret.TurretPower;
import frc.robot.utils.Trajectories;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Trench8L extends SequentialCommandGroup {

  /** Creates a new Trench8M. */
  public Trench8L() {
    addCommands(
        // First reset the sensors and odometry.
        new ResetGyro(),
        new ResetOdometry(Trajectories.trench8L.POSE1),
        new ShooterVelocity(4000, true),

        // Second, aim the turret close to the target.
        new TurretAngle(20.0),

        // Auto aim the turret and fire.
        new AutoShoot(true).withTimeout(5.0),

        // End the auto-aiming and shooting.
        new AutoShoot(false),
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                // Drive down the trench.
                new FollowTrajectory(Trajectories.trench8L.DOWN_TRENCH, new Rotation2d()),
                new FollowTrajectory(Trajectories.trench8L.BALL1, new Rotation2d()),
                new FollowTrajectory(Trajectories.trench8L.BALL2, new Rotation2d())),
            // Begin intaking balls
            new AutoIntake(true)),
        new FollowTrajectory(Trajectories.trench8L.TO_SHOOT, new Rotation2d()),
        new AutoShoot(true).withTimeout(5.0),
        new AutoShoot(false),
        new ParallelCommandGroup(
            new SpinIntake(0.0, false),
            new SpinHopper(0.0, false),
            new TurretPower(0.0),
            new ShooterVelocity(Constants.shooter.IDLE_VEL, true)));
  }
}

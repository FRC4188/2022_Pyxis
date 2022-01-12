// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.hood.SetPosition;
import frc.robot.commands.hopper.AutoMagazine;
import frc.robot.commands.shooter.FormulaRPM;
import frc.robot.commands.shooter.ShooterVelocity;
import frc.robot.commands.turret.FollowTarget;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShoot extends ParallelCommandGroup {
  /** Creates a new AutoShoot. */
  public AutoShoot(boolean cont) {
    addCommands(
        new AutoMagazine(cont),
        // new SpinIntake(0.75, cont),
        new FollowTarget(cont),
        new FormulaRPM(cont),
        new SetPosition(cont));
  }

  public AutoShoot(double rpm, boolean cont) {
    addCommands(
        new AutoMagazine(cont),
        // new SpinIntake(0.75, cont),
        new FollowTarget(cont),
        new ShooterVelocity(rpm, cont),
        new SetPosition(cont));
  }
}

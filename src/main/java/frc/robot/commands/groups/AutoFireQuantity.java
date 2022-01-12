// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.hood.SetPosition;
import frc.robot.commands.intake.SpinIntake;
import frc.robot.commands.shooter.FormulaRPM;
import frc.robot.commands.turret.FollowTarget;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoFireQuantity extends ParallelDeadlineGroup {
  /** Creates a new AutoFireQuantity. */
  public AutoFireQuantity(int shots) {
    super(new InstantCommand()); // AutoMagazine(shots));
    addCommands(
        new SpinIntake(0.75, true),
        new FollowTarget(true),
        new FormulaRPM(true),
        new SetPosition(true));
  }
}

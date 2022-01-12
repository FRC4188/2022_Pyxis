// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.hopper.LoadHopper;
import frc.robot.commands.intake.SpinIntake;
import frc.robot.subsystems.intake.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoIntake extends ParallelCommandGroup {
  /** Creates a new AutoIntake. */
  public AutoIntake(boolean cont) {
    addCommands(
        new LoadHopper(cont),
        new SpinIntake(0.5, cont),
        new RunCommand(() -> Intake.getInstace().setRaised(!cont)));
  }
}

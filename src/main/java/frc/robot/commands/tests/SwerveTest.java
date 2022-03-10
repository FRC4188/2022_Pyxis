// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tests;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.InterruptSubsystem;
import frc.robot.commands.drive.CrabSet;
import frc.robot.subsystems.drive.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwerveTest extends SequentialCommandGroup {
  /** Creates a new SwerveTest. */
  public SwerveTest() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new CrabSet(2.0, 90.0).withTimeout(2.0),
      new CrabSet(3.0, -90.0).withTimeout(2.0),
      new InterruptSubsystem(Swerve.getInstance())
    );
  }
}

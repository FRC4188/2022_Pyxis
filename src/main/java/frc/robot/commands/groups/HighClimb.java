// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climber.ActivePull;
import frc.robot.commands.climber.ActivePush;
import frc.robot.commands.climber.PassivePull;
import frc.robot.commands.climber.PassivePush;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HighClimb extends SequentialCommandGroup {
  /** Creates a new HighClimb. */
  public HighClimb() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ActivePush(),
      new ActivePull(),
      new PassivePush(),
      new ActivePush(),
      //new PassivePull(),
      new PassivePull(),
      new ActivePull(),
      new PassivePush()
    );
  }
}

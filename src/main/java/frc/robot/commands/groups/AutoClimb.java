// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.climber.ActivePull;
import frc.robot.commands.climber.ActivePush;
import frc.robot.commands.climber.PassivePull;
import frc.robot.commands.climber.PassivePush;
import frc.robot.commands.climber.PitchWaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoClimb extends SequentialCommandGroup {
  /** Creates a new AutoClimb. */
  public AutoClimb() {
    addCommands(
      new PassivePush(),
      new ActivePush(),
      //new PitchWaitCommand(3.427734375),
      new PassivePull(),
      //new PitchWaitCommand(2.9443359375),
      new WaitCommand(1.0),
      new ActivePull(),
      new PassivePush()
    );
  }
}

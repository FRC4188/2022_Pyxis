// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.climber.ActivePosition;
import frc.robot.commands.climber.ImpatientPassive;
import frc.robot.commands.sensors.PitchGreaterThanCommand;
import frc.robot.commands.sensors.PitchLessThanCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MonkeyBar extends SequentialCommandGroup {
  /** Creates a new AutoClimb. */
  public MonkeyBar() {
    addCommands(
      new ImpatientPassive(true),
      new ActivePosition(0.7),
      new PitchGreaterThanCommand(44.0),
      new ActivePosition(Constants.climber.MAX_HEIGHT),
      new ImpatientPassive(false),
      new PitchLessThanCommand(37.0),
      new ActivePosition(0.0)
    );
  }
}

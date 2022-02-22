// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.climber.ActivePosition;
import frc.robot.commands.climber.ImpatientPassive;
import frc.robot.commands.climber.PatientPassive;
import frc.robot.commands.sensors.PitchWaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoClimb extends SequentialCommandGroup {
  /** Creates a new AutoClimb. */
  public AutoClimb() {
    addCommands(
      new ImpatientPassive(true),
      new ActivePosition(0.7),
      new PitchWaitCommand(3.427734375),
      new ActivePosition(Constants.climber.PUSH_POSITION),
      new ImpatientPassive(false),
      new PitchWaitCommand(2.9443359375),
      new ActivePosition(0.8),
      new ImpatientPassive(true),
      new ActivePosition(0.2),
      new PatientPassive(false),
      new ActivePosition(0.0)
    );
  }
}

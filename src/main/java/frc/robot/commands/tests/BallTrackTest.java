// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tests;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.indexer.SpinIndexer;
import frc.robot.commands.intake.SpinIntake;
import frc.robot.commands.trigger.PushTrigger;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BallTrackTest extends SequentialCommandGroup {
  /** Creates a new BallTrackTest. */
  public BallTrackTest() {
    addCommands(
      new ParallelCommandGroup(
        new SpinIntake(12.0, true),
        new SpinIndexer(8.0),
        new PushTrigger(12.0)
      ),
      new WaitCommand(2.0),
      new ParallelCommandGroup(
        new SpinIntake(12.0, true),
        new SpinIndexer(8.0),
        new PushTrigger(12.0)
      )
    );
  }
}

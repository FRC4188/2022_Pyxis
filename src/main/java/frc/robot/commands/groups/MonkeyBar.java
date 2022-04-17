// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.climber.ActivePosition;
import frc.robot.commands.climber.ImpatientPassive;
import frc.robot.commands.drive.CrabSet;
import frc.robot.commands.indexer.SpinIndexer;
import frc.robot.commands.intake.SpinIntake;
import frc.robot.commands.sensors.PitchGreaterThanCommand;
import frc.robot.commands.sensors.PitchLessThanCommand;
import frc.robot.commands.shooter.HoodAngle;
import frc.robot.commands.shooter.ShooterVelocity;
import frc.robot.commands.trigger.PushTrigger;
import frc.robot.commands.turret.SetToAngle;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MonkeyBar extends ParallelDeadlineGroup {
  /** Creates a new AutoClimb. */
  public MonkeyBar() {
    super(new SequentialCommandGroup(
      new ActivePosition(0.0),
      new ImpatientPassive(true),
      new ActivePosition(0.6),
      new PitchGreaterThanCommand(40.0),
      new ActivePosition(Constants.climber.MAX_HEIGHT + 0.03),
      new ImpatientPassive(false),
      new PitchLessThanCommand(38.0),
      new ActivePosition(0.3)
    ));

    addCommands(
        new HoodAngle(()-> 0.0),
        new SetToAngle(-180.0),
        new ShooterVelocity(() -> 0.0),
        new SpinIntake(0.0, true),
        new SpinIndexer(0.0),
        new PushTrigger(0.0),
        new CrabSet(0.0, 0.0)
    );
  }
}

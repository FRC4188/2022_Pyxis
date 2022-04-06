// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.trigger.PreShooter;

public class LoadBalls extends CommandBase {

  Indexer indexer = Indexer.getInstance();
  PreShooter trigger = PreShooter.getInstance();

  /** Creates a new LoadBalls. */
  public LoadBalls() {
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!trigger.getTop() || !trigger.getBottom()) indexer.set(12.0);
    else indexer.set(0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

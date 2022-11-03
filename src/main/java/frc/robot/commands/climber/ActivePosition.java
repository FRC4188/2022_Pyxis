// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.climber.Climber;

public class ActivePosition extends CommandBase {

  private Climber climber = Climber.getInstance();
  private double pos;
  /** Creates a new ActivePosition. */
  public ActivePosition(double position) {
    addRequirements(climber);
    this.pos = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setBrake(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setActivePosition(pos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setActiveVolts(0.0);
    climber.setBrake(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(climber.getActivePositionA() - pos) < Constants.climber.ACTIVE_TOLERANCE
        && Math.abs(climber.getActivePositionB() - pos) < Constants.climber.ACTIVE_TOLERANCE;
  }
}

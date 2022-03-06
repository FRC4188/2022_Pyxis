// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.trigger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.trigger.PreShooter;

public class PushTrigger extends CommandBase {

  private PreShooter trigger = PreShooter.getInstance();
  private double volts;

  /** Creates a new PushTrigger. */
  public PushTrigger(double volts) {
    addRequirements(trigger);
    this.volts = volts;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    trigger.setVoltage(volts);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    trigger.setVoltage(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

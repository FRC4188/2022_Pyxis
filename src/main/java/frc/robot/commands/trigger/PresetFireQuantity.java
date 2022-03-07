// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.trigger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.trigger.PreShooter;

public class PresetFireQuantity extends CommandBase {

  private PreShooter trigger = PreShooter.getInstance();
  private Shooter shooter = Shooter.getInstance();
  private Hood hood = Hood.getInstance();

  private double angle, rpm;
  private int quantity;

  private boolean lastTop = false;

  /** Creates a new PresetFire. */
  public PresetFireQuantity(double angle, double rpm, int quantity) {
    addRequirements(trigger);

  this.quantity = quantity;
  this.angle = angle;
  this.rpm = rpm;
}

// Called when the command is initially scheduled.
@Override
public void initialize() {}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
  if (shooter.isReady(rpm) && hood.isReady(angle)) trigger.set(12.0);
  else trigger.set(0.0);

  if (!trigger.getTop() && lastTop) quantity --;

  lastTop = trigger.getTop();
}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
  trigger.set(0.0);
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
  return quantity == 0;
}
}

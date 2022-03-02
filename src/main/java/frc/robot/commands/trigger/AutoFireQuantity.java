// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.trigger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.trigger.Trigger;

public class AutoFireQuantity extends CommandBase {
  Trigger trigger = Trigger.getInstance();
  Shooter shooter = Shooter.getInstance();
  Hood hood = Hood.getInstance();

  private int quantity = 0;
  private boolean lastTop = false;

  /** Creates a new AutoFire. */
  public AutoFireQuantity(int quantity) {
    addRequirements(trigger);

    this.quantity = quantity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.isReady() && hood.isReady()) trigger.set(12.0);
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

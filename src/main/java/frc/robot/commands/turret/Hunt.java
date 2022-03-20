// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.sensors.Sensors;
import frc.robot.subsystems.turret.Turret;

public class Hunt extends CommandBase {
  Turret turret = Turret.getInstance();

  boolean reverse;

  /** Creates a new WrapAround. */
  public Hunt(boolean reverse) {
    addRequirements(turret);
    this.reverse = reverse;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.setVolts(reverse ? -12.0 : 12.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setVolts(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Sensors.getInstance().getHasTarget();
  }
}

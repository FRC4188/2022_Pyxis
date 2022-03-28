// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.turret.Turret;

public class TurretAngleWait extends CommandBase {

  private double desired;
  private Turret turret = Turret.getInstance();

  /** Creates a new TurretAngleWait. */
  public TurretAngleWait(double desired) {
    this.desired = desired;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(turret.getPosition() - desired) <= Constants.turret.ANGLE_TOLERANCE;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.turret.Turret;

public class OppositeAim extends CommandBase {

  private Turret turret = Turret.getInstance();
  private double target = 0.0;

  /** Creates a new OppositeAim. */
  public OppositeAim() {
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Math.abs(turret.getPosition()) < Math.abs(turret.getPosition() - 180.0)) target = 180.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.setAngle(target);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(turret.getPosition() - target) < Constants.turret.POS_TOLERANCE;
  }
}

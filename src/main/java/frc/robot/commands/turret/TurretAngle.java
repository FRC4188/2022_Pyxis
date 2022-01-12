/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.turret.Turret;

public class TurretAngle extends CommandBase {

  Turret turret = Turret.getInstance();
  double angle;
  /** Creates a new TurretAngle. */
  public TurretAngle(double angle) {
    addRequirements(turret);
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.setAngle(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(turret.getPosition() - angle) < Constants.turret.POS_TOLERANCE
        && Math.abs(turret.getVelocity()) < Constants.turret.VEL_TOLERANCE;
  }
}

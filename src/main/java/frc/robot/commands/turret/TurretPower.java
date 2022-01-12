/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.Turret;

public class TurretPower extends CommandBase {
  Turret turret = Turret.getInstance();
  double power;
  private boolean cont;
  /** Creates a new TurretPower. */
  public TurretPower(double power, boolean cont) {
    addRequirements(turret);

    this.power = power;
    this.cont = cont;
  }

  /** Creates a new TurretPower which will run until interrupted. */
  public TurretPower(double power) {
    this(power, true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.set(power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !cont;
  }
}

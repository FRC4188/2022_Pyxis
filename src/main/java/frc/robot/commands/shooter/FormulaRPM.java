// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.sensors.Sensors;
import frc.robot.subsystems.shooter.Shooter;

public class FormulaRPM extends CommandBase {

  private Shooter shooter = Shooter.getInstance();
  private boolean cont;

  /** Creates a new FormulaRPM. */
  public FormulaRPM(boolean cont) {
    addRequirements(shooter);

    this.cont = cont;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setVelocity(Sensors.getInstance().getFormulaRPM());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setPercentage(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !cont;
  }
}

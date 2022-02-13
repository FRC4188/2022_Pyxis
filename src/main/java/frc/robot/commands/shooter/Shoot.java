// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LowerShooter;
import frc.robot.subsystems.UpperShooter;
import frc.robot.subsystems.sensors.Sensors;

public class Shoot extends CommandBase {

  Sensors sensors = Sensors.getInstance();
  UpperShooter upper = UpperShooter.getInstance();
  LowerShooter lower = LowerShooter.getInstance();

  /** Creates a new Shoot. */
  public Shoot() {
    addRequirements(upper, lower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(sensors.getTX()) < 10.0) {
      upper.setVelocity(sensors.getFormulaRPM() / 2.0);
      lower.setVelocity(sensors.getFormulaRPM());
    } else {
      upper.setVelocity(0.0);
      lower.setVelocity(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    upper.setVelocity(0.0);
    lower.setVelocity(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

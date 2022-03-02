// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.shooter;
import frc.robot.subsystems.shooter.Shooter;

public class FindHoodZeros extends CommandBase {

  Shooter shooter = Shooter.getInstance();

  LinearFilter aFilter;

  double aVolts = 0.0;

  /** Creates a new FindZeros. */
  public FindHoodZeros() {
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    shooter.setHoodVolts(-2.0);

    aFilter = LinearFilter.movingAverage(20);
    aFilter.calculate(-1.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setHoodVolts(-1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setHoodVolts(0.0);
    shooter.resetHood();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(aFilter.calculate(shooter.getHoodVelocity())) < 0.05;
  }
}

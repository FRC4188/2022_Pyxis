// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.hood.Hood;

public class FindHoodZeros extends CommandBase {

  Hood hood = Hood.getInstance();

  LinearFilter aFilter;

  double aVolts = 0.0;

  /** Creates a new FindZeros. */
  public FindHoodZeros() {
    addRequirements(hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    aFilter = LinearFilter.movingAverage(20);
    aFilter.calculate(-1.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.setVolts(-1.0);
    hood.resetPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(aFilter.calculate(hood.getVelocity())) < 0.05;
  }
}

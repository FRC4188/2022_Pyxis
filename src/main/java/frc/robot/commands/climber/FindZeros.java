// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;

public class FindZeros extends CommandBase {

  Climber climber = Climber.getInstance();

  LinearFilter aFilter;
  LinearFilter bFilter;

  double aVolts = 0.0;
  double bVolts = 0.0;

  /** Creates a new FindZeros. */
  public FindZeros() {
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setBrake(false);
    climber.setActiveVolts(aVolts);

    aFilter = LinearFilter.movingAverage(10);
    bFilter = LinearFilter.movingAverage(10);
    aFilter.calculate(-1.0);
    bFilter.calculate(-1.0);
    aVolts = -2.0;
    bVolts = -2.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double aVel = aFilter.calculate(climber.getActiveVelocityA());
    double bVel = bFilter.calculate(climber.getActiveVelocityB());

    if (Math.abs(aVel) < 0.005) {
      climber.resetActiveA(0.0);
      aVolts = 0.0;
    }
    if (Math.abs(bVel) < 0.005) {
      climber.resetActiveB(0.0);
      bVolts = 0.0;
    }

    climber.setActiveVolts(aVolts, bVolts);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setActiveVolts(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return aVolts == 0.0 && bVolts == 0.0;
  }
}

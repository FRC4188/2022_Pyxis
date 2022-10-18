// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.hood.Hood;

public class FindHoodZeros extends CommandBase {

  Hood hood = Hood.getInstance();

  /** Creates a new FindZeros. */
  public FindHoodZeros() {
    addRequirements(hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hood.setServoing(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hood.setVolts(-1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.setVolts(0.0);
    hood.resetPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hood.atZero();
  }
}

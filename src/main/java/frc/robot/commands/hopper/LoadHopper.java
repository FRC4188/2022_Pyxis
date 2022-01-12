// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.hopper.Hopper;

public class LoadHopper extends CommandBase {

  Hopper hopper = Hopper.getInstance();
  private boolean cont;

  /** Creates a new LoadHopper. */
  public LoadHopper(boolean cont) {
    addRequirements(hopper);

    this.cont = cont;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hopper.set(0.0);
    // if (!hopper.getTopBeam()) hopper.set(0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !cont;
  }
}

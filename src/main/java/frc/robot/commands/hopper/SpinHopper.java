/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.hopper.Hopper;

public class SpinHopper extends CommandBase {

  Hopper hopper = Hopper.getInstance();
  double percentage;
  boolean cont;

  /** Creates a new SpinHopper. */
  public SpinHopper(double percentage, boolean cont) {
    addRequirements(hopper);

    this.percentage = percentage;
    this.cont = cont;
  }

  /** Creates a new SpinHopper which will run until interrupted. */
  public SpinHopper(Hopper hopper, double percentage) {
    this(percentage, true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hopper.set(percentage);
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

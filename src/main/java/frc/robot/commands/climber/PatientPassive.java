// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;

public class PatientPassive extends CommandBase {
  
  private Climber climber = Climber.getInstance();
  private boolean pushing;

  public PatientPassive(boolean pushing) {
    addRequirements(climber);
    this.pushing = pushing;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setPassivePosition(pushing);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pushing ? climber.getPassivePosition() == 1 : climber.getPassivePosition() == -1;
  }
}

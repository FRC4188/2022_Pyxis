// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.climber.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetActiveB extends InstantCommand {

  Climber climber = Climber.getInstance();

  private double position;

  public ResetActiveB(double position) {
    addRequirements(climber);
    this.position = position;
  }

  public ResetActiveB() {
    this(0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.resetActiveB(position);
  }
}

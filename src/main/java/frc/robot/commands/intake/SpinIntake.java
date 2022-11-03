// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;

public class SpinIntake extends CommandBase {
  private Intake intake = Intake.getInstance();
  private double power;
  private boolean raised;

  public SpinIntake(double power, boolean raised) {
    addRequirements(intake);

    this.power = power;
    this.raised = raised;
  }

  /** Creates a new SpinIntake. */
  public SpinIntake(double power) {
    this(power, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.raise(raised);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setVoltage(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.set(0.0);
    intake.raise(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

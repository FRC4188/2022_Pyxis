/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;

public class SpinIntake extends CommandBase {
  private Intake intake = Intake.getInstace();
  private double power;
  private boolean cont;

  /** Creates a new SpinIntake. */
  public SpinIntake(double power, boolean cont) {
    addRequirements(intake);

    this.power = power;
    this.cont = cont;
  }

  /** Creates a new SpinIntake which will run until interrupted. */
  public SpinIntake(double power) {
    this(power, true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.set(power);
    System.out.println("intake running");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.set(0.0);
    System.out.println("intake done");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !cont;
  }
}

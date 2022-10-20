// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.sensors.Sensors;

public class TrackBalls extends CommandBase {
  private Swerve swerve = Swerve.getInstance();
  private Sensors sensors = Sensors.getInstance();

  private DoubleSupplier xInput;
  private DoubleSupplier yInput;

  /** Creates a new TrackBalls. */
  public TrackBalls(DoubleSupplier xInput, DoubleSupplier yInput) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
    this.xInput = xInput;
    this.yInput = yInput;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.drive(yInput.getAsDouble(), xInput.getAsDouble(), swerve.getTrackingPID().calculate(sensors.getClosestBallAngle(), 0.0) / (2.0 * Math.PI));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Odometry;
import frc.robot.subsystems.drive.Swerve;

public class Reverse extends CommandBase {

  private Swerve drive = Swerve.getInstance();
  private Odometry odometry = Odometry.getInstance();
  private double distance;
  private double startX;
  private double startY;

  /** Creates a new Reverse. */
  public Reverse(double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

    this.distance = distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startX = odometry.getPose().getX();
    startY = odometry.getPose().getY();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.drive(-0.3, 0.0, 0.0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(0.0, 0.0, 0.0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.hypot(odometry.getPose().getX() - startX, odometry.getPose().getY() - startY)
        >= distance;
  }
}

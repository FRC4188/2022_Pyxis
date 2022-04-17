// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.sensors.Sensors;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrackBalls extends CommandBase {

  Swerve drive = Swerve.getInstance();
  Sensors sensors = Sensors.getInstance();

  DoubleSupplier xVel, yVel;

  PIDController pid = new PIDController(-0.15, 0.0, 0.0);

  /** Creates a new TrackBalls. */
  public TrackBalls(DoubleSupplier xVel, DoubleSupplier yVel) {
    this.xVel = xVel;
    this.yVel = yVel;

    addRequirements(drive);
  }

  @Override
  public void initialize() {

  }

  public void execute() {
    double rotCorrection = pid.calculate(sensors.getClosestBallAngle(), 0.0);
    drive.setChassisSpeeds(new ChassisSpeeds(xVel.getAsDouble() * 2.0, yVel.getAsDouble() * 2.0, rotCorrection));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
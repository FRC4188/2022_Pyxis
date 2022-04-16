// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.sensors.Sensors;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrackBalls extends PIDCommand {

  static Swerve drive = Swerve.getInstance();
  static Sensors sensors = Sensors.getInstance();

  /** Creates a new TrackBalls. */
  public TrackBalls(DoubleSupplier xVel, DoubleSupplier yVel) {
    super(
        // The controller that the command will use
        new PIDController(-0.0025, 0, 0),
        // This should return the measurement
        () -> sensors.getClosestBallAngle(),
        // This should return the setpoint (can also be a constant)
        () -> 0.0,
        // This uses the output
        output -> {
          drive.drive(yVel.getAsDouble(), xVel.getAsDouble(), output, 0.0, false, false);
        });
    addRequirements(drive);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
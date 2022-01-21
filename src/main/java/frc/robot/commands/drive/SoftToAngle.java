// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SoftToAngle extends ProfiledPIDCommand {
  /** Creates a new SoftToAngle. */
  public SoftToAngle(double angle) {
    super(
        // The ProfiledPIDController used by the command
        Constants.drive.thetaPID.thetaPID,
        // This should return the measurement
        () -> Swerve.getInstance().getPose().getRotation().getDegrees(),
        // This should return the goal (can also be a constant)
        () -> new TrapezoidProfile.State(angle, 0.0),
        // This uses the output
        (output, setpoint) -> {
          Swerve.getInstance().setChassisSpeeds(new ChassisSpeeds(0.0, 0.0, output));
        },
        Swerve.getInstance()
      );
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

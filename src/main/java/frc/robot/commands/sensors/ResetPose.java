package frc.robot.commands.sensors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.sensors.Sensors;

public class ResetPose extends InstantCommand {

  private Pose2d pose;

  public ResetPose() {
    this(new Pose2d());
  }

  public ResetPose(Pose2d pose) {
    this.pose = pose;
    addRequirements(Swerve.getInstance());
  }

  @Override
  public void initialize() {
    Sensors.getInstance().setPigeonAngle(pose.getRotation().getDegrees());
    Swerve.getInstance().setPose(pose);
    Swerve.getInstance().setRotSetpoint(0.0);
  }
}

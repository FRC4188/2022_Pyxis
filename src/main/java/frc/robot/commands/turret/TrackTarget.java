package frc.robot.commands.turret;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.sensors.Sensors;
import frc.robot.subsystems.turret.Turret;

public class TrackTarget extends CommandBase {
  private Turret turret = Turret.getInstance();

  
  /** Creates a new TrackTarget. */
  public TrackTarget() {
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double goodAngle = -(Units.radiansToDegrees(Math.atan2(-Swerve.getInstance().getPose().getY(), -Swerve.getInstance().getPose().getX())) - Sensors.getInstance().getRotation().getDegrees() + 180);
    // if (turret.getPosition() - goodAngle > 90) {
    //   goodAngle = (goodAngle + 360) % 360 - 180;
    // } else if (turret.getPosition() - goodAngle < -90) {
    //   goodAngle = (goodAngle - 360) % 360 + 180;
    // }
    goodAngle = goodAngle % 360.0;
    turret.setAngle(goodAngle + Sensors.getInstance().getOffsetAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
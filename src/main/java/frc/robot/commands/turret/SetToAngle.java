package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.turret.Turret;

public class SetToAngle extends CommandBase {
  private Turret turret = Turret.getInstance();

  private double angle;

  /** Creates a new SetToAngle. */
  public SetToAngle(double angle) {
    addRequirements(turret);
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.setAngle(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(turret.getPosition() - angle) < Constants.turret.ANGLE_TOLERANCE
        && Math.abs(turret.getVelocity()) < 5.0;
  }
}

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.sensors.Sensors;
import frc.robot.subsystems.shooter.Shooter;

public class Shoot extends CommandBase {

  Sensors sensors = Sensors.getInstance();
  Shooter shooter = Shooter.getInstance();

  /** Creates a new Shoot. */
  public Shoot() {
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.isReady()) {
      shooter.setVelocity(sensors.getFormulaRPM());
    } else {
      shooter.setVelocity(0.0);
    }
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

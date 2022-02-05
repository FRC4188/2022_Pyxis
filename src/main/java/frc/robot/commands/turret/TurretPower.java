package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.Turret;

public class TurretPower extends CommandBase {
  Turret turret = Turret.getInstance();
  double power;
  
  public TurretPower(double power) {
    addRequirements(turret);

    this.power = power;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.set(power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
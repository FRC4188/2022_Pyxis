package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;
import java.util.function.DoubleSupplier;

public class ActiveVolts extends CommandBase {

  private Climber climber = Climber.getInstance();
  private DoubleSupplier voltsA;
  private DoubleSupplier voltsB;

  /** Creates a new ActiveVolts. */
  public ActiveVolts(DoubleSupplier voltsA, DoubleSupplier voltsB) {
    addRequirements(climber);
    this.voltsA = voltsA;
    this.voltsB = voltsB;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setBrake(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setActiveVolts(voltsA.getAsDouble(), voltsB.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setActiveVolts(0.0);
    climber.setBrake(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

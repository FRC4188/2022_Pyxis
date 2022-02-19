package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.climber.Climber;

public class ActivePull extends CommandBase {
  /** Creates a new ActivePull. */

  Climber climber = Climber.getInstance();

  public ActivePull() {
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setBrake(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setActivePosition(Constants.climber.PULL_POSITION);
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
    return Math.max(climber.getActivePositionA() - Constants.climber.PULL_POSITION, climber.getActivePositionB() - Constants.climber.PULL_POSITION) <= Constants.climber.ACTIVE_TOLERANCE;
  }
}

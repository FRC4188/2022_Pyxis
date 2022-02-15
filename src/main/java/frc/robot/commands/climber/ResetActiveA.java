package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.climber.Climber;

public class ResetActiveA extends InstantCommand {

  Climber climber = Climber.getInstance();

  private double position;

  /** Creates a new ResetActiveA. */
  public ResetActiveA(double position) {
    addRequirements(climber);
    this.position = position;
  }

  public ResetActiveA() {
    this(0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.resetActiveA(position);
  }
}

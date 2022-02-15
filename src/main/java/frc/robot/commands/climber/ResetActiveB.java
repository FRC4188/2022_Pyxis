package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.climber.Climber;

public class ResetActiveB extends InstantCommand {

  Climber climber = Climber.getInstance();

  private double position;

  /** Creates a new ResetActiveB */
  public ResetActiveB(double position) {
    addRequirements(climber);
    this.position = position;
  }

  public ResetActiveB() {
    this(0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.resetActiveB(position);
  }
}

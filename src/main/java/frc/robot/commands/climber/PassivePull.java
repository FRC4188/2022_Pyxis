package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.climber.Climber;

public class PassivePull extends InstantCommand {
  /** Creates a new PassivePull. */

  Climber climber = Climber.getInstance();
  public PassivePull() {
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setPassivePosition(true);
  }
}

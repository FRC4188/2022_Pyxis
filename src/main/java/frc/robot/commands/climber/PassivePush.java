package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.climber.Climber;

public class PassivePush extends InstantCommand {

  Climber climber = Climber.getInstance();
  /** Creates a new PassivePush. */
  public PassivePush() {
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setPassivePosition(false);
  }
}

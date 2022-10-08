package frc.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Pnuematics;

public class Deploy extends InstantCommand{
    Pnuematics pnuematics = Pnuematics.getInstance();

    //private boolean isRaised;
    public Deploy(boolean isRaised) {
        addRequirements(pnuematics);
    }


    @Override

    public void initialize() {
        pnuematics.raise(true);
    }

    // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pnuematics.raise(false);
  }
}

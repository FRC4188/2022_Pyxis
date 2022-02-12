package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class LowerIntake extends CommandBase {

    private final Intake intake;

    /**
     * Constructs new LowwerIntake command to fire intake solenoids to lowered position.
     *
     * @param intake - Intake subsystem to use.
     */
    public LowerIntake(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intake.lower();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
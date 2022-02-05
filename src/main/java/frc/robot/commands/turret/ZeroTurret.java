package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.Turret;

/**
 * Resets turret encoders.
 */
public class ZeroTurret extends CommandBase {

    private final Turret turret;

    /**
     * Constructs new ZeroTurret command to reset turret encoders.
     *
     * @param turret - Turret subsystem to use.
     */
    public ZeroTurret(Turret turret) {
        addRequirements(turret);
        this.turret = turret;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        turret.resetEncoder();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }

}

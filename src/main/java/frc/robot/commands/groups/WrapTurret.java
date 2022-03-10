package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.turret.Hunt;
import frc.robot.commands.turret.SetToAngle;
import frc.robot.commands.turret.TrackTarget;
import frc.robot.subsystems.turret.Turret;

public class WrapTurret extends SequentialCommandGroup {
  Turret turret = Turret.getInstance();
  /** Creates a new WrapTurret. */
  public WrapTurret() {
    if (Math.abs(turret.getPosition() - Constants.turret.MIN_ANGLE) < Math.abs(turret.getPosition() - Constants.turret.MIN_ANGLE))
      addCommands(
        new SetToAngle(Constants.turret.MIN_ANGLE + 180.0),
        new Hunt(false),
        new TrackTarget()
      );
    else
      addCommands(
        new SetToAngle(Constants.turret.MAX_ANGLE - 180.0),
        new Hunt(true),
        new TrackTarget()
      );
  }
}

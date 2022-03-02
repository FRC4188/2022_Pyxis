package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.indexer.LoadBalls;
import frc.robot.commands.shooter.HoodAngle;
import frc.robot.commands.shooter.ShooterVelocity;
import frc.robot.commands.trigger.AutoFireQuantity;
import frc.robot.commands.turret.TrackTarget;
import frc.robot.subsystems.sensors.Sensors;

public class AutoShootQuantity extends ParallelDeadlineGroup {
  /** Creates a new AutoShootQuantity. */
  public AutoShootQuantity(int quantity) {
    super(new AutoFireQuantity(quantity));
    addCommands(
      new ShooterVelocity(() -> Sensors.getInstance().getFormulaRPM()),
      new TrackTarget(true),
      new HoodAngle(() -> Sensors.getInstance().getFormulaAngle()),
      new LoadBalls()
    );
  }
}

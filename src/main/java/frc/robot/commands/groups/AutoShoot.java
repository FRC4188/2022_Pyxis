package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.indexer.LoadBalls;
import frc.robot.commands.shooter.HoodAngle;
import frc.robot.commands.shooter.ShooterVelocity;
import frc.robot.commands.trigger.AutoFire;
import frc.robot.commands.turret.TrackTarget;
import frc.robot.subsystems.sensors.Sensors;

public class AutoShoot extends ParallelCommandGroup {
  /** Creates a new AutoShoot. */
  public AutoShoot() {
    addCommands(
      new ShooterVelocity(() -> Sensors.getInstance().getFormulaRPM()),
      new AutoFire(),
      new TrackTarget(true),
      new HoodAngle(() -> Sensors.getInstance().getFormulaAngle()),
      new LoadBalls()
    );
  }
}
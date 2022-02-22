package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.indexer.LoadBalls;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.trigger.AutoFire;
import frc.robot.commands.turret.TrackTarget;

public class AutoShoot extends ParallelCommandGroup {
  /** Creates a new AutoShoot. */
  public AutoShoot() {
    addCommands(
      new LoadBalls(),
      new AutoFire(),
      new TrackTarget(true),
      new Shoot()
    );
  }
}
package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.indexer.LoadBalls;
import frc.robot.commands.indexer.SpinIndexer;
import frc.robot.commands.intake.SpinIntake;
import frc.robot.commands.shooter.HoodAngle;
import frc.robot.commands.shooter.ShooterVelocity;
import frc.robot.commands.trigger.AutoFire;
import frc.robot.commands.turret.TrackTarget;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.sensors.Sensors;

public class AutoShoot extends ParallelCommandGroup {
  /** Creates a new AutoShoot. */
  public AutoShoot(boolean raised) {
    addCommands(
      new ShooterVelocity(() -> Sensors.getInstance().getFormulaRPM()),
      new AutoFire(),
      new TrackTarget(),
      new HoodAngle(() -> Sensors.getInstance().getFormulaAngle()),
      new LoadBalls()
    );
  }
}
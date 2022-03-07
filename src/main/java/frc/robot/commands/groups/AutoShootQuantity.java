package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.indexer.LoadBalls;
import frc.robot.commands.shooter.HoodAngle;
import frc.robot.commands.shooter.ShooterVelocity;
import frc.robot.commands.trigger.AutoFireQuantity;
import frc.robot.commands.turret.TrackTarget;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.sensors.Sensors;

public class AutoShootQuantity extends ParallelDeadlineGroup {
  /** Creates a new AutoShootQuantity. */
  public AutoShootQuantity(int quantity, boolean intaking) {
    super(new AutoFireQuantity(quantity));
    addCommands(
      new ShooterVelocity(() -> Sensors.getInstance().getFormulaRPM()),
      new TrackTarget(),
      new HoodAngle(() -> Sensors.getInstance().getFormulaAngle()),
      new LoadBalls(),
      new RunCommand(() -> {
          Intake.getInstance().raise(!intaking);
          if (intaking) Intake.getInstance().setVoltage(12.0);
        }, Intake.getInstance())
    );
  }
}

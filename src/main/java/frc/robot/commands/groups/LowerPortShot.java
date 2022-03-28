// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.indexer.LoadBalls;
import frc.robot.commands.shooter.HoodAngle;
import frc.robot.commands.shooter.ShooterVelocity;
import frc.robot.commands.trigger.PushTrigger;
import frc.robot.commands.turret.SetToAngle;
import frc.robot.commands.turret.TurretAngleWait;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LowerPortShot extends ParallelCommandGroup {
  /** Creates a new LowerPortShot. */
  public LowerPortShot() {
    addCommands(
      new SetToAngle(-180.0),
      new ShooterVelocity(() -> 1200.0),
      new HoodAngle(() -> 45.0),
      new TurretAngleWait(-180.0).andThen(new PushTrigger(12.0)),
      new LoadBalls()
    );
  }
}

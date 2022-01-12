// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.groups.AutoShoot;
import frc.robot.commands.turret.TurretAngle;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Shoot3 extends SequentialCommandGroup {
  /** Creates a new Shoot3. */
  public Shoot3() {
    addCommands(
        new TurretAngle(180.0), new AutoShoot(true).withTimeout(5.0).andThen(new AutoShoot(false)));
  }
}

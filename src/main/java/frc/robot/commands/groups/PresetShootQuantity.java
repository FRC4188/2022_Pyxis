// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.indexer.LoadBalls;
import frc.robot.commands.shooter.HoodAngle;
import frc.robot.commands.shooter.ShooterVelocity;
import frc.robot.commands.trigger.PresetFireQuantity;
import frc.robot.commands.turret.TrackTarget;
import frc.robot.subsystems.intake.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PresetShootQuantity extends ParallelCommandGroup {
  /** Creates a new PresetShootQuantity.*/
  public PresetShootQuantity(double angle, double velocity, int quantity, boolean intaking) {
    super(new PresetFireQuantity(angle, velocity, quantity));
    addCommands(
      new ShooterVelocity(() -> velocity),
      new TrackTarget(),
      new HoodAngle(() -> angle),
      new LoadBalls(),
      new RunCommand(() -> {
          Intake.getInstance().raise(!intaking);
          if (intaking) Intake.getInstance().setVoltage(12.0);
        }, Intake.getInstance())
    );
  }
}

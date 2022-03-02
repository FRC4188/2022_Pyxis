// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;

public class HoodAngle extends CommandBase {

  private Hood hood = Hood.getInstance();

  DoubleSupplier angle;

  /** Creates a new HoodAngle. */
  public HoodAngle(DoubleSupplier angle) {
    this.angle = angle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void initialize() {
    hood.setPosition(angle.getAsDouble());
  }
}

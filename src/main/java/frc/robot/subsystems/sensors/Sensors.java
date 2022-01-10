// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensors;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sensors extends SubsystemBase {

  private static Sensors instance = null;

  public static synchronized Sensors getInstance() {
    if (instance == null) instance = new Sensors();
    return instance;
  }

  /** Creates a new Sensors. */
  private Sensors() {
    CommandScheduler.getInstance().registerSubsystem(this);
  }

  @Override
  public void periodic() {
  }

  public Rotation2d getRotation() {
    return new Rotation2d();
  }

  public void setRotation(Rotation2d rotation) {
  }
}

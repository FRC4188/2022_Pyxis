// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensors;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sensors extends SubsystemBase {

  private static Sensors instance = null;

  public static synchronized Sensors getInstance() {
    if (instance == null) instance = new Sensors();
    return instance;
  }

  private Pigeon pigeon = new Pigeon(31);

  /** Creates a new Sensors. */
  private Sensors() {
    CommandScheduler.getInstance().registerSubsystem(this);
  }

  @Override
  public void periodic() {
  }

  public Rotation2d getRotation() {
    return pigeon.get();
  }

  public double getOmegaRad() {
    return pigeon.getOmegaRadians();
  }

  public void setRotation(Rotation2d rotation) {
    pigeon.setFusedHeading(rotation.getDegrees());
  }
}

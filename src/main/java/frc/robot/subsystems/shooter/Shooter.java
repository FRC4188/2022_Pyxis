// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.sensors.Sensors;

public class Shooter extends SubsystemBase {
  private static Shooter instance;
  public static synchronized Shooter getInstance() {
    if (instance == null) instance = new Shooter();
    return instance;
  }

  private Flywheel flywheel = Constants.Shooter.FLYWHEEL;

  private Sensors sensors = Sensors.getInstance();

  /** Creates a new Shooter. */
  private Shooter() {
    setVelocity(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setVelocity(double velocity) {
    flywheel.setVelocity(velocity);
  }

  public double getVelocity() {
    return flywheel.getVelocity();
  }

  public boolean isReady() {
    return Math.abs(getVelocity() - sensors.getCalculatedRPM()) < 75;
  }
}

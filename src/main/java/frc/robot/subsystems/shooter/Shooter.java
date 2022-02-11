// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private static Shooter instance;

  public static synchronized Shooter getInstance() {
    if (instance == null) instance = new Shooter();
    return instance;
  }
  private UpperShooter upper = new UpperShooter();
  private LowerShooter lower = new LowerShooter();

  private Notifier notifier = new Notifier(() -> updateShuffleboard());

  /** Creates a new Shooter. */
  public Shooter() {
    SmartDashboard.putNumber("Set Velocity", 0.0);

    notifier.startPeriodic(0.1);
  }

  public void updateShuffleboard() {
    SmartDashboard.putNumber("Upper Velocity", getVelocities()[0]);
    SmartDashboard.putNumber("Lower Velocity", getVelocities()[1]);
    SmartDashboard.putNumber("Average Velocity", getVelocities()[2]);

  }

  public void setVelocity(double velocity) {
    upper.setVelocity(velocity);
    //lower.setVelocity(velocity + 500);
  }

  public void setVolatge(double voltage) {
    upper.setVoltage(voltage);
    lower.setVoltage(voltage);
  }

  public double[] getVelocities() {
    double average = (upper.getVelocity() + lower.getVelocity()) / 2;
    double[] velocities = {upper.getVelocity(), lower.getVelocity(), average};
    return velocities;
  }

  public double[] getTemperatures() {
    double[] temps = {upper.getTemperature(), lower.getTemperature()};
    return temps;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

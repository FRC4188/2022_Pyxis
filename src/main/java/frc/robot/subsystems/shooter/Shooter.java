// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
    CommandScheduler.getInstance().registerSubsystem(this);

    startNotifier();

    SmartDashboard.putNumber("Set Velocity", 0);
  }

  private void updateShuffleboard() {
    SmartDashboard.putNumber("Average Velocity", getVelocity());
    SmartDashboard.putNumberArray("Shooter Temperatures", getTemperatures());
  }

  private void startNotifier() {
    notifier.startPeriodic(0.3);
  }

  public void set(double percentage) {
    upper.set(percentage);
    lower.set(percentage);
  }

  public void setVelocity(double velocity) {
    upper.setVelocity(velocity / 2);
    lower.setVelocity(velocity);
  }

  public double getVelocity() {
    return (upper.getVelocity() + lower.getVelocity()) / 2;
  }

  public double[] getTemperatures() {
    double[] temperatures = {upper.getTemperature(), lower.getTemperature()};
    return temperatures;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

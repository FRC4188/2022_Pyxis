// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.DoubleSolenoid;

public class Climber extends SubsystemBase {

  private static Climber instance = null;
  public static synchronized Climber getInstance() {
    if (instance == null) instance = new Climber();
    return instance;
  }

  private ActiveHook active = new ActiveHook();
  private PassiveHook passive = new PassiveHook();

  private DoubleSolenoid brake = Constants.devices.brake;

  private Notifier dashboardLoop = new Notifier(() -> updateDashboard());

  /** Creates a new Climber. */
  private Climber() {
    CommandScheduler.getInstance().registerSubsystem(this);

    dashboardLoop.startPeriodic(0.1);

    SmartDashboard.putNumber("Climber Set Voltage", 0.0);
    setBrake(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void updateDashboard() {
    SmartDashboard.putNumber("Active Climber A Position", active.getPositionA());
    SmartDashboard.putNumber("Active Climber B Position", active.getPositionB());
    SmartDashboard.putBoolean("Passive Climber Set State", passive.getSet());
    SmartDashboard.putNumber("Temperature Motor A", active.getMotorATemp());
    SmartDashboard.putNumber("Temperature Motor B", active.getMotorBTemp());
  }

  public double getActivePositionA() {
    return active.getPositionA();
  }

  public double getActivePositionB() {
    return active.getPositionB();
  }

  public double getActiveVelocityA() {
    return active.getVelocityA();
  }

  public double getActiveVelocityB() {
    return active.getVelocityB();
  }

  public boolean getPassiveSet() {
    return passive.getSet();
  }

  public boolean getBrake() {
    return brake.get();
  }

  public void setActivePosition(double output) {
    active.setPosition(output);
  }

  public void setPassivePosition(boolean output) {
    passive.setPosition(output);
  }

  public void setBrake(boolean state) {
    brake.set(state);
    active.motorBrakes(state);
  }

  public void setActiveVolts(double volts) {
    active.setA(volts);
    active.setB(volts);
  }

  public void setActiveVolts(double A, double B) {
    active.setA(A);
    active.setB(B);
  }

  public void resetActiveA(double position) {
    active.resetPositionA(position);
  }

  public void resetActiveB(double position) {
    active.resetPositionB(position);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private static Climber instance = null;
  public static synchronized Climber getInstance() {
    if (instance == null) instance = new Climber();
    return instance;
  }

  private ActiveHook active = new ActiveHook();
  private PassiveHook passive = new PassiveHook();

  private DigitalInput limitA = new DigitalInput(channel)

  private Notifier dashboardLoop = new Notifier(() -> updateDashboard());

  /** Creates a new Climber. */
  private Climber() {
    dashboardLoop.startPeriodic(0.1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void updateDashboard() {
    SmartDashboard.putNumber("Active Climber A Position", active.getPositionA());
    SmartDashboard.putNumber("Active Climber B Position", active.getPositionB());
    SmartDashboard.putNumber("Active Climber Average Position", getActivePosition());
    SmartDashboard.putBoolean("Passive Climber State", passive.getPosition());
    SmartDashboard.putNumber("Temperature Motor A", active.getMotorATemp());
    SmartDashboard.putNumber("Temperature Motor B", active.getMotorBTemp());
  }

  public double getActivePosition() {
    return (active.getPositionA()  + active.getPositionB()) / 2;
  }

  public boolean getPassivePosition() {
    return passive.getPosition();
  }

  public void setActivePosition(double output) {
    active.setPosition(output);
  }

  public void setPassivePosition(boolean output) {
    passive.setPosition(output);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.climber.ResetActiveA;
import frc.robot.commands.climber.ResetActiveB;

public class Climber extends SubsystemBase {

  private static Climber instance = null;
  public static synchronized Climber getInstance() {
    if (instance == null) instance = new Climber();
    return instance;
  }

  private ActiveHook active = new ActiveHook();
  private PassiveHook passive = new PassiveHook();

  private DigitalInput lowLimitA = new DigitalInput(Constants.climber.LOW_LIMIT_A_ID);
  private DigitalInput lowLimitB = new DigitalInput(Constants.climber.LOW_LIMIT_B_ID);

  private DigitalInput hookLimitA = new DigitalInput(Constants.climber.HOOK_LIMIT_A_ID);
  private DigitalInput hookLimitB = new DigitalInput(Constants.climber.HOOK_LIMIT_B_ID);

  private Solenoid brake = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.climber.BRAKE_ID);

  private Notifier dashboardLoop = new Notifier(() -> updateDashboard());

  /** Creates a new Climber. */
  private Climber() {
    CommandScheduler.getInstance().registerSubsystem(this);

    dashboardLoop.startPeriodic(0.1);

    new Trigger(() -> lowLimitA.get()).whileActiveOnce(new ResetActiveA(), false);
    new Trigger(() -> lowLimitB.get()).whileActiveOnce(new ResetActiveB(), false);

    SmartDashboard.putNumber("Climber Set Voltage", 0.0);
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

  public double getActivePositionA() {
    return active.getPositionA();
  }

  public double getActivePositionB() {
    return active.getPositionB();
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

  public void setBrake(boolean state) {
    brake.set(!state);
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

  public boolean isHooked() {
    return hookLimitA.get() && hookLimitB.get();
  }
}

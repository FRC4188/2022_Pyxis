// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
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

  private DigitalInput limitA = new DigitalInput(Constants.climber.LIMIT_A_ID);
  private DigitalInput limitB = new DigitalInput(Constants.climber.LIMIT_B_ID);

  private Solenoid brake = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.climber.BRAKE_ID);

  private Notifier dashboardLoop = new Notifier(() -> updateDashboard());

  /** Creates a new Climber. */
  private Climber() {
    CommandScheduler.getInstance().registerSubsystem(this);

    dashboardLoop.startPeriodic(0.1);

    new Trigger(() -> limitA.get()).whileActiveOnce(new ResetActiveA(), false);
    new Trigger(() -> limitB.get()).whileActiveOnce(new ResetActiveB(), false);
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
    brake.set(state);
  }

  public void setActiveVolts(double volts) {
    active.setA(volts);
    active.setB(volts);
  }
  public boolean getLimitA(){
    return limitA.get();
  }
  public boolean getLimitB(){
    return limitB.get();
  }
  public boolean getLimitAB(){
    return limitA.get() && limitB.get();
  }

  public void resetActiveA(double position) {
    active.resetPositionA(position);
  }

  public void resetActiveB(double position) {
    active.resetPositionB(position);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.sensors.Sensors;
import frc.robot.utils.math.Derivative;
import frc.robot.utils.motors.CSPMotor;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Hood extends SubsystemBase {

  private static Hood instance = null;
  public static synchronized Hood getInstance() {
    if (instance == null) instance = new Hood();
    return instance;
  }

  CSPMotor motor = Constants.devices.hoodMotor;
  PIDController pid = new PIDController(Constants.shooter.hood.kP, Constants.shooter.hood.kI, Constants.shooter.hood.kD);
  ArmFeedforward ff = new ArmFeedforward(0.0, Constants.shooter.hood.kCos, 0.0);
  private double position = 0.0;

  DigitalInput limit = new DigitalInput(4);

  boolean servoing = true;

  private Debouncer readyFilter = new Debouncer(0.05, DebounceType.kBoth);

  private Derivative readyError = new Derivative(0.0);
  
  private Hood() {
      motor.reset();
      motor.setInverted(false);
      motor.setBrake(false);
      motor.setRamp(0.0);

      new Trigger(() -> !limit.get()).whenActive(new InstantCommand(() -> resetPosition()));
  }

  @Override
  public void periodic() {
      if (servoing) setVolts(pid.calculate(getPosition(), position) + ff.calculate(Math.toRadians(position + 9.8), 0.0));
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("Hood Angle", getPosition());
    SmartDashboard.putNumber("Hood Temperature", getTemp());
    SmartDashboard.putNumber("Hood Current", motor.getCurrent());
    SmartDashboard.putBoolean("Hood is ready", isReady());
  }

  public void setPosition(double position) {
      servoing = true;
      if (position > Constants.shooter.hood.MAX) position = Constants.shooter.hood.MAX;
      else if (position < Constants.shooter.hood.MIN) position = Constants.shooter.hood.MIN;
      this.position = position;
  }

  public void resetPosition() {
      motor.reset();
  }

  public void setServoing(boolean servoing) {
      this.servoing = servoing;
  }

  public void setVolts(double volts) {
      setVolts(volts, true);
  }

  public void setVolts(double volts, boolean softLimit) {
      if (softLimit) {
          if (getPosition() <= Constants.shooter.hood.MIN && volts < 0.0) motor.set(0.0);
          else if (getPosition() >= Constants.shooter.hood.MAX && volts > 0.0) motor.set(0.0);
          else motor.set(-volts / RobotController.getBatteryVoltage());
      } else {
          motor.set(0.0);
      }
  }

  public double getPosition() {
      return -motor.getPosition() * Constants.shooter.hood.CONVERSION;
  }

  public double getTemp() {
      return motor.getTemperature();
  }

  public double getVelocity() {
      return motor.getVelocity();
  }

  public boolean isReady(double angle) {
    return readyFilter.calculate(Math.abs(getPosition() - angle) < 1.0);
  }

  public boolean isReady() {
    return isReady(Sensors.getInstance().getFormulaAngle());
  }

public boolean atZero() {
    return !limit.get();
}
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.sensors.Sensors;
import frc.robot.utils.motors.CSPMotor;

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
  Notifier shuffle;
  
  private Hood() {
      motor.reset();
      motor.setInverted(false);
      motor.setBrake(false);
      motor.setRamp(0.0);

      shuffle = new Notifier(() -> updateDashboard());
      shuffle.startPeriodic(0.2);
  }

  @Override
  public void periodic() {
      setVolts(pid.calculate(getPosition(), position) + ff.calculate(Math.toRadians(position + 9.8), 0.0));
  }

  private void updateDashboard() {
    SmartDashboard.putNumber("Hood Angle", getPosition());
    SmartDashboard.putNumber("Hood Temperature", getTemp());
    SmartDashboard.putBoolean("Hood is ready", isReady());
  }

  public void setPosition(double position) {
      if (position > Constants.shooter.hood.MAX) position = Constants.shooter.hood.MAX;
      else if (position < Constants.shooter.hood.MIN) position = Constants.shooter.hood.MIN;
      this.position = position;
  }

  public void resetPosition() {
      motor.reset();
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
    return Math.abs(getPosition() - angle) < 1.25;
  }

  public boolean isReady() {
    return isReady(Sensors.getInstance().getFormulaAngle());
  }
}

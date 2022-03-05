// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {

  private static Hood instance = null;
  public static synchronized Hood getInstance() {
    if (instance == null) instance = new Hood();
    return instance;
  }

  CANSparkMax motor;
  RelativeEncoder encoder;
  //CANCoder encoder;
  PIDController pid = new PIDController(Constants.shooter.hood.kP, Constants.shooter.hood.kI, Constants.shooter.hood.kD);
  ArmFeedforward ff = new ArmFeedforward(0.0, Constants.shooter.hood.kCos, 0.0);
  private double position = 0.0;
  Notifier shuffle;
  
  private Hood() {
      motor = new CANSparkMax(Constants.shooter.hood.MOTOR_ID, MotorType.kBrushless);
      encoder = motor.getEncoder();
      motor.restoreFactoryDefaults();
      motor.setInverted(true);
      motor.setIdleMode(IdleMode.kBrake);
      motor.clearFaults();
      motor.setOpenLoopRampRate(0.0);
      //motor.setOpenLoopRampRate(0.0125);

      encoder.setPositionConversionFactor(Constants.shooter.hood.CONVERSION);
      encoder.setVelocityConversionFactor(Constants.shooter.hood.CONVERSION);
      resetPosition();

      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 20);

      shuffle = new Notifier(() -> updateDashboard());
      shuffle.startPeriodic(0.1);
      /*
      encoder = new CANCoder(Constants.shooter.hood.ENCODER_ID);
      encoder.configFactoryDefault();
      encoder.setPosition(0.0);
      encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
      encoder.configMagnetOffset(Constants.shooter.hood.ZERO);
      encoder.clearStickyFaults();
      encoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 65000);
      */
  }

  @Override
  public void periodic() {
      setVolts(pid.calculate(getPosition(), position) + ff.calculate(Math.toRadians(position + 9.8), 0.0));
  }

  private void updateDashboard() {
    SmartDashboard.putNumber("Hood Angle", getPosition());
    SmartDashboard.putNumber("Hood Temperature", getTemp());
  }

  public void setPosition(double position) {
      if (position > Constants.shooter.hood.MAX) position = Constants.shooter.hood.MAX;
      else if (position < Constants.shooter.hood.MIN) position = Constants.shooter.hood.MIN;
      this.position = position;
  }

  public void resetPosition() {
      encoder.setPosition(0.0);
  }

  public void setVolts(double volts) {
      setVolts(volts, true);
  }

  public void setVolts(double volts, boolean softLimit) {
      if (softLimit) {
          if (getPosition() <= Constants.shooter.hood.MIN && volts < 0.0) motor.set(0.0);
          else if (getPosition() >= Constants.shooter.hood.MAX && volts > 0.0) motor.set(0.0);
          else motor.set(volts / RobotController.getBatteryVoltage());
      } else {
          motor.set(0.0);
          //motor.set(volts / RobotController.getBatteryVoltage());
      }
  }

  public double getPosition() {
      return encoder.getPosition();
  }

  public double getTemp() {
      return motor.getMotorTemperature();
  }

  public double getVelocity() {
      return encoder.getVelocity();
  }

  public boolean isReady() {
    return Math.abs(getPosition() - position) < (1.0) && Math.abs(getVelocity()) < 1.0;
  }
}

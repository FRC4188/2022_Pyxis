// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.sensors.Sensors;

public class Turret extends SubsystemBase {
  private static Turret instance;

  public static synchronized Turret getInstance() {
    if (instance == null) instance = new Turret();
    return instance;
  }

  private CANSparkMax motor = new CANSparkMax(Constants.turret.MOTOR_ID, MotorType.kBrushless);
  private RelativeEncoder encoder = motor.getEncoder();

  private PIDController pid = new PIDController(Constants.turret.kP, Constants.turret.kI, Constants.turret.kD);

  Notifier notifier = new Notifier(() -> updateShuffleboard());

  /** Creates a new Turret. */
  public Turret() {
    CommandScheduler.getInstance().registerSubsystem(this);

    initialize();

    startNotifier();
  }

  private void initialize() {
    encoder.setPositionConversionFactor(Constants.turret.ENCODER_TO_DEGREES);
    encoder.setPosition(0.0);

    motor.setIdleMode(IdleMode.kBrake);

    motor.setClosedLoopRampRate(1.0);
    motor.setOpenLoopRampRate(1.0);
    motor.clearFaults();
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 40000);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20000);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);
  }

  private void startNotifier() {
    notifier.startPeriodic(1.0);
  }

  private void updateShuffleboard() {
    SmartDashboard.putNumber("Turret Motor Temp", getTemperature());
    SmartDashboard.putNumber("Turret Position", getPosition());
  }

  public void setPIDs(double kP, double kI, double kD) {
    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
  }

  public void set(double percent) {
    if (getPosition() < Constants.turret.MIN_ANGLE && percent < 0.0 || 
      getPosition() > Constants.turret.MAX_ANGLE && percent > 0.0) {
      motor.set(0.0);
    } else {
      motor.set(percent);
    }
  }

  public void setVolts(double volts) {
    set(volts / RobotController.getBatteryVoltage());
  }

  public void setAngle(double angle) {
    setVolts(pid.calculate(getPosition(), angle));
  }

  public void trackTarget(boolean cont) {
    double angle = Sensors.getInstance().getTX();

    setAngle(getPosition() + angle);
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public double getTemperature() {
    return motor.getMotorTemperature();
  }

  public boolean getIsAimed() {
    double angle = Sensors.getInstance().getTX();
    boolean aimed = (Math.abs(angle) < Constants.turret.ANGLE_TOLERANCE);
    return aimed;
  }
}
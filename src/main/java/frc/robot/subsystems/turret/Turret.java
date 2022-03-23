// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.sensors.Sensors;
import frc.robot.utils.motors.CSPMotor;
import lib4188.data.Data;
import lib4188.data.DataHandler;
import lib4188.data.Data.Key;
import lib4188.subsystem.CSPSubsystem;

public class Turret extends CSPSubsystem {
  private static Turret instance;

  public static synchronized Turret getInstance() {
    if (instance == null) instance = new Turret();
    return instance;
  }

  private CSPMotor motor = Constants.devices.turretMotor;

  private PIDController targetPID = new PIDController(Constants.turret.TkP, Constants.turret.TkI, Constants.turret.TkD);
  private PIDController positionPID = new PIDController(Constants.turret.PkP, Constants.turret.PkI, Constants.turret.PkD/*, new Constraints(Constants.turret.MAX_VEL, Constants.turret.MAX_ACCEL)*/);

  public enum Keys implements Key {
    POSITION,
    TEMPERATURE,
    /**1 for true, 0 for false. */
    AIMED
  }

  private Turret() {
    super();
  }

  @Override
  public void startup() {
    motor.reset();
    motor.setBrake(false);
    motor.setRamp(0.1);
    motor.set(0.0);
  }

  @Override
  public void updateDashboard() {
    Data datum = DataHandler.getDatum(Keys.POSITION);
    SmartDashboard.putNumber(datum.getLabel(), datum.getDatum().doubleValue());
    datum = DataHandler.getDatum(Keys.TEMPERATURE);
    SmartDashboard.putNumber(datum.getLabel(), datum.getDatum().doubleValue());
    datum = DataHandler.getDatum(Keys.AIMED);
    SmartDashboard.putBoolean(datum.getLabel(), datum.getDatum().intValue() == 1);
  }

  @Override
  public void updateData() {
    DataHandler.addDatum(Keys.POSITION, new Data("Turret position", getPosition()));
    DataHandler.addDatum(Keys.TEMPERATURE, new Data("Turret temperature", getTemperature()));
    DataHandler.addDatum(Keys.AIMED, new Data("Turret is aimed", getIsAimed() ? 1 : 0));
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
    setVolts(positionPID.calculate(getPosition(), angle));
  }

  public void trackTarget() {
    if (Sensors.getInstance().getHasTarget()) setVolts(targetPID.calculate(0.0, Sensors.getInstance().getTX()/* + Sensors.getInstance().getOffsetAngle()*/) + -Swerve.getInstance().getChassisSpeeds().omegaRadiansPerSecond * 6.0);
    else set(0.0);
  }

  private double getPosition() {
    return motor.getPosition() * Constants.turret.ENCODER_TO_DEGREES;
  }

  private double getTemperature() {
    return motor.getTemperature();
  }

  private boolean getIsAimed() {
    double angle = Sensors.getInstance().getTX();
    boolean aimed = (Math.abs(angle) < Constants.turret.ANGLE_TOLERANCE) && Sensors.getInstance().getHasTarget();
    return aimed;
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.trigger;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Trigger extends SubsystemBase {

  private static Trigger instance = null;
  public static synchronized Trigger getInstance() {
    if (instance == null) instance = new Trigger();
    return instance;
  }

  private WPI_TalonFX motor;

  /** Creates a new Trigger. */
  private Trigger() {
    motor = new WPI_TalonFX(Constants.indexer.TRIGGER_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double power) {
    motor.set(power);
  }

  public void setVoltage(double volts) {
    set(volts / RobotController.getBatteryVoltage());
  }
}

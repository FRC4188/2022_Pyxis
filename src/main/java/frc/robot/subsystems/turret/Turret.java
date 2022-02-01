// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import frc.robot.Constants;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.turret;

public class Turret extends SubsystemBase {

  private static Turret instance = null;

  public static synchronized Turret getInstance() {
    if (instance == null) instance = new Turret();

    return instance;
  }

  CANSparkMax motor = new CANSparkMax(0, MotorType.kBrushless);
  PIDController PID = new PIDController(turret.kP, turret.kI, turret.kD);

  /** Creates a new Turret. */
  private Turret() {
    CommandScheduler.getInstance().registerSubsystem(this);

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void motorInit()
  {
    PID.setP(turret.kP);
    PID.setI(turret.kI);
    PID.setD(turret.kD);
  }

  public void set(double amount)
  {

  }

  public void setAngle(double angle)
  {
    
  }

  /*
  * Outputs the temperature of the motor in the form of a double.
  */
  public void getTemp()
  {
    SmartDashboard.putNumber("Temperature: ", motor.getMotorTemperature());
  }

  public void getPosition()
  {

  }
}

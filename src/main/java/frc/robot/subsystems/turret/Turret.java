// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import frc.robot.Constants;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;

public class Turret extends SubsystemBase {

  private static Turret instance = null;

  public static synchronized Turret getInstance() {
    if (instance == null) instance = new Turret();

    return instance;
  }

  CANSparkMax motor = new CANSparkMax(11, MotorType.kBrushless);
  RelativeEncoder encoder = motor.getEncoder();
  ProfiledPIDController pid = new ProfiledPIDController(Constants.turret.kD, Constants.turret.kI, Constants.turret.kP, new Constraints(Constants.turret.MAX_VELOCITY, Constants.turret.MAX_ACCEL));

  private Notifier shuffleboard = new Notifier(() -> updateShuffleboard());

  /** Creates a new Turret. */
  private Turret() {
    CommandScheduler.getInstance().registerSubsystem(this);

    motorInit();
    resetEncoder();


    
    SmartDashboard.putNumber("set P", Constants.turret.kP);
    SmartDashboard.putNumber("set I", Constants.turret.kI);
    SmartDashboard.putNumber("set D", Constants.turret.kD);

    shuffleboard.startPeriodic(0.4);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Configuresh the motors of the turret
  private void motorInit()
  {
    pid.setP(Constants.turret.kP);
    pid.setI(Constants.turret.kI);
    pid.setD(Constants.turret.kD);

    motor.setIdleMode(IdleMode.kBrake);
  }

  public void setPID(double kP, double kI, double kD){
    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
  }
  /*
  * Sets the turret motor to the amount you want (has to be between -1.0 and 1.0).
  * Stops the turret from moving when it reaches its min or max angle.
  *
  * @param amount The amount you want the turret to move.
  */
  public void set(double amount)
  {
    motor.set(amount);
    // if (getPosition() >= Constants.turret.MAX_ANGLE)
    // motor.set(0.0);
    // else if (getPosition() <= Constants.turret.MIN_ANGLE)
    // motor.set(0.0);
  }

  public void setAngle(double angle)
  {
   set(pid.calculate(getPosition(), angle));

  }
  
  // Resets the encoder's current position to 0
  public void resetEncoder()
  {
    encoder.setPosition(0.0);
  }

  // Reloads the information on the Shuffleboard
  private void updateShuffleboard()
  {


    SmartDashboard.putNumber("Position", getPosition());
    SmartDashboard.putNumber("Temperature", getTemp());
    SmartDashboard.putNumber("Velocity", getVelocity());
  }

  /*
  * Outputs the temperature of the motor
  *
  * @return Temperature of motor in celsius (?)
  */
  public double getTemp()
  {
    return motor.getMotorTemperature();
  }

  /* 
  * Outputs the current position of the turret
  *
  * @return Current position of turret in degrees
  */
  public double getPosition()
  {
    return encoder.getPosition() * Constants.turret.encoderToDegrees;
  }

  /* 
  * Outputs the velocity of the turret
  *
  * @return Velocity of turret in degrees/second
  */
  public double getVelocity()
  {
    return encoder.getVelocity() * Constants.turret.encoderToDegrees / 60.0;
  }
}
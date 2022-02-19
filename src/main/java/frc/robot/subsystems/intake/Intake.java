// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
//import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.DoubleSolenoid;

public class Intake extends SubsystemBase {
  private static Intake intake;

  public static synchronized Intake getInstance () {
    if (intake == null) intake= new Intake();
    return intake;
  }

  private CANSparkMax intakeMotor = new CANSparkMax(41, MotorType.kBrushless);
  private RelativeEncoder encoder = intakeMotor.getEncoder();
  private ProfiledPIDController pid = new ProfiledPIDController(1.0, 0.0, 0.0, new Constraints(3, 1));

  private double set;

  Notifier shuffle = new Notifier(() -> updateSuffleboard());

  public Intake(){
    CommandScheduler.getInstance().registerSubsystem(this);
    initIntake();
    resetEncoders();
    intakeMotor.restoreFactoryDefaults();

    //SmartDashboard.putNumber("set P", Constants.intake.kP);
    //SmartDashboard.putNumber("set I", Constants.intake.kI);
    //SmartDashboard.putNumber("set D", Constants.intake.kD);

    shuffle.startPeriodic(0.4);
  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Temperature", testMotor.getMotorTemperature());
    //SmartDashboard.putNumber("Intake Motor Temp", intakeMotor.getMotorTemperature());
    
  }

  public void updateSuffleboard() {
    SmartDashboard.putNumber("Motor Velocity", getVelocity());
    SmartDashboard.putNumber("Motor Temperature", getTemperature());
    SmartDashboard.putNumber("Set Value", set);
    }

 public void initIntake(){
   intakeMotor.setClosedLoopRampRate(1.0);
   intakeMotor.setOpenLoopRampRate(1.0);
   intakeMotor.setIdleMode(IdleMode.kBrake);
  }

  public void resetEncoders() {
    encoder.setPosition(0);
  }

  public void setPID(double kP, double kI, double kD) {
    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
  }

  public void set(double percentage){
    intakeMotor.set(percentage);
  }

  public void setVoltage(double voltage) {
    set(voltage / RobotController.getBatteryVoltage());
  }

  public void setVelocity(double velocity) {
    set = pid.calculate(getVelocity(), velocity);
    set(set);
  }

  private double getTemperature() {
  return intakeMotor.getMotorTemperature();
}

public double getVelocity() {
  return encoder.getVelocity() / 3;
}

public void raise(boolean engaged) {

}

  // shows intake motor temp on smart dashboard
 
}

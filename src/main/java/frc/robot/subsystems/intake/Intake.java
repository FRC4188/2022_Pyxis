// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
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

  private WPI_TalonFX intakeMotor = new WPI_TalonFX(Constants.intake.MOTOR_ID);
  private ProfiledPIDController pid = new ProfiledPIDController(1.0, 0.0, 0.0, new Constraints(3, 1));
  private DoubleSolenoid piston;

  private double set;

  Notifier shuffle = new Notifier(() -> updateSuffleboard());

  public Intake(){
    CommandScheduler.getInstance().registerSubsystem(this);
    initIntake();
    resetEncoders();
    intakeMotor.configFactoryDefault();
    SmartDashboard.putNumber("Intake Set Voltage", 0.0);

    piston = new DoubleSolenoid(Constants.intake.SOLENOID_A_ID, Constants.intake.SOLENOID_B_ID);

    //SmartDashboard.putNumber("set P", Constants.intake.kP);
    //SmartDashboard.putNumber("set I", Constants.intake.kI);
    //SmartDashboard.putNumber("set D", Constants.intake.kD);

    shuffle.startPeriodic(0.4);

    raise(false);
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
   intakeMotor.configClosedloopRamp(1.0);
   intakeMotor.configOpenloopRamp(1.0);
   intakeMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void resetEncoders() {
    intakeMotor.setSelectedSensorPosition(0);
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
  return intakeMotor.getTemperature();
}

public double getVelocity() {
  return intakeMotor.getSelectedSensorVelocity() / 3;
}

public void raise(boolean engaged) {
  piston.set(engaged);
}

public boolean isRaised() {
  return piston.get();
}

  // shows intake motor temp on smart dashboard
 
}

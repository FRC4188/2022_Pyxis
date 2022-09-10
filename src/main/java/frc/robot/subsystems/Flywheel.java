// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase {
  private static Flywheel instance;

  public static synchronized Flywheel getInstance() {
    if (instance == null) instance = new Flywheel();
    return instance;
  }

  private LinearSystem<N1, N1, N1> shooterPlant = LinearSystemId.identifyVelocitySystem(Constants.FLYWHEEL.kV, Constants.FLYWHEEL.kA);
  private KalmanFilter<N1, N1, N1> filter = new KalmanFilter<>(Nat.N1(), Nat.N1(), shooterPlant, 
    VecBuilder.fill(3.0), 
    VecBuilder.fill(0.01), 
    0.020);
  private LinearQuadraticRegulator<N1, N1, N1> regulator = new LinearQuadraticRegulator<>(shooterPlant, 
    //no idea
    VecBuilder.fill(8.0), 
    //this stays
    VecBuilder.fill(12.0),
    //this stays
    0.020);
  private LinearSystemLoop<N1, N1, N1> loop = new LinearSystemLoop<>(shooterPlant, regulator, filter, 12.0, 0.020);

  private CANSparkMax motor = new CANSparkMax(9, MotorType.kBrushless);
  private RelativeEncoder encoder = motor.getEncoder();
  
  private Notifier shuffle = new Notifier(() -> updateShuffleboard());

  /** Creates a new ShooterSystemID. */
  public Flywheel() {
    CommandScheduler.getInstance().registerSubsystem(this);
    encoder.setVelocityConversionFactor(1.0);

    loop.reset(VecBuilder.fill(getVelocity()));

    SmartDashboard.putNumber("Set Flywheel Velocity", 0.0);
    SmartDashboard.putNumber("Set Percentage", 0.0);
    
    shuffle.startPeriodic(0.1);
  }

  public void openNotifier() {
    shuffle.startPeriodic(0.1);
  }

  private void updateShuffleboard() {
    SmartDashboard.putNumber("Flywheel Velocity (RPM)", getVelocity());
    SmartDashboard.putNumber("Flywheel Estimated Velocity (RPM)", getEstimatedVelocity());
    SmartDashboard.putNumber("Get Next Reference", getNextReference());
    SmartDashboard.putNumber("Get Input Voltage", getInputVoltage());

  }

  public void set(double percentage) {
    motor.set(percentage);
  }

  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  public void setVelocity(double velocity) {
    loop.setNextR(VecBuilder.fill(velocity));
    loop.correct(VecBuilder.fill(getVelocity()));
    loop.predict(0.02);
    
    setVoltage(loop.getU(0));
  }

  public double getVelocity() {
    return encoder.getVelocity();
  }

  public double getTemperature() {
    return motor.getMotorTemperature();
  }

  public double getInputVoltage() {
    return loop.getU(0);
  }

  public double getEstimatedVelocity() {
    return loop.getXHat(0);
  }   

  public double getNextReference() {
    return loop.getNextR(0);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

/**
 * 
  private LinearSystem<N1, N1, N1> shooterPlant = LinearSystemId.identifyVelocitySystem(Constants.FLYWHEEL.kV, Constants.FLYWHEEL.kA);
  private KalmanFilter<N1, N1, N1> filter = new KalmanFilter<>(Nat.N1(), Nat.N1(), shooterPlant, 
    VecBuilder.fill(0.0001), 
    VecBuilder.fill(60.1), 
    0.02);
  private LinearQuadraticRegulator<N1, N1, N1> regulator = new LinearQuadraticRegulator<>(shooterPlant, 
    VecBuilder.fill(8.0), 
    VecBuilder.fill(12.0),
    0.020);
  private LinearSystemLoop<N1, N1, N1> loop = new LinearSystemLoop<>(shooterPlant, regulator, filter, 12.0, 0.020);
 */
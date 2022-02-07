// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class UpperShooter extends SubsystemBase {
  private static UpperShooter instance;

  public static synchronized UpperShooter getInstance() {
    if (instance == null) instance = new UpperShooter();
    return instance;
  }

  private LinearSystem<N1, N1, N1> shooterPlant = LinearSystemId.identifyVelocitySystem(Constants.shooter.upper.kV, Constants.shooter.upper.kA);
  private KalmanFilter<N1, N1, N1> filter = new KalmanFilter<>(Nat.N1(), Nat.N1(), shooterPlant, VecBuilder.fill(Constants.shooter.upper.SYSTEM_STDEV), VecBuilder.fill(Constants.shooter.upper.ENC_STDEV), 0.2);
  private LinearQuadraticRegulator<N1, N1, N1> regulator = new LinearQuadraticRegulator<>(shooterPlant, VecBuilder.fill(Constants.shooter.upper.QELMS), VecBuilder.fill(Constants.shooter.upper.RELMS), 0.020);
  private LinearSystemLoop<N1, N1, N1> loop = new LinearSystemLoop<>(shooterPlant, regulator, filter, 12.0, 0.020);

  private WPI_TalonFX motor = new WPI_TalonFX(10);
  
  private Notifier shuffle = new Notifier(() -> updateShuffleboard());


  private double velocity = 0.0;

  /** Creates a new ShooterSystemID. */
  public UpperShooter() {
    CommandScheduler.getInstance().registerSubsystem(this);

    loop.reset(VecBuilder.fill(getVelocity()));

    motor.setInverted(true);
    
    motor.setNeutralMode(NeutralMode.Coast);
    motor.setSelectedSensorPosition(0.0);
    motor.configOpenloopRamp(Constants.shooter.upper.RAMP);

    openNotifier();
  }

  public void openNotifier() {
    shuffle.startPeriodic(0.1);
  }

  private void updateShuffleboard() {
    SmartDashboard.putNumber("Upper Temp", getTemperature());
    SmartDashboard.putNumber("Upper Vel", getVelocity());
    SmartDashboard.putNumber("Upper Volts", motor.get() * RobotController.getBatteryVoltage());
  }

  public void set(double percentage) {
    motor.set(percentage);
  }

  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  public void setRPM(double velocity) {
    this.velocity = velocity;
  }

  public void setMpS(double speed) {
    setRPM(speed);
  }

  public double getVelocity() {
    return (motor.getSelectedSensorVelocity() * 600.0) / (2048.0 * Constants.shooter.upper.GEARING);
  }

  public double getPosition() {
    return (motor.getSelectedSensorPosition()) / (2048.0 * Constants.shooter.upper.GEARING);
  }

  public double getTemperature() {
    return motor.getTemperature();
  }

  @Override
  public void periodic() {
    loop.reset(VecBuilder.fill(getVelocity() / 60.0));
    loop.setNextR(VecBuilder.fill(velocity / 60.0));
    loop.correct(VecBuilder.fill(getVelocity() / 60.0));
    loop.predict(0.02);
    
    setVoltage(loop.getU(0));
  }
}

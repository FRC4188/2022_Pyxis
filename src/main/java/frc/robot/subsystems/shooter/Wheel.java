// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Wheel {

  private WPI_TalonFX leader;
  private WPI_TalonFX follower;
  private PIDController pid = new PIDController(Constants.shooter.kP, Constants.shooter.kI, Constants.shooter.kD);
  
  private Notifier shuffle = new Notifier(() -> updateShuffleboard());

  private double velocity = 0.0;

  protected Wheel(int leaderID, int followerID) {
    leader = new WPI_TalonFX(leaderID);
    follower = new WPI_TalonFX(followerID);
    
    leader.clearStickyFaults();
    follower.clearStickyFaults();

    leader.setInverted(false);
    follower.setInverted(true);

    leader.setNeutralMode(NeutralMode.Coast);
    leader.setSelectedSensorPosition(0.0);
    leader.configOpenloopRamp(Constants.shooter.RAMP);
    leader.configClosedloopRamp(Constants.shooter.RAMP);

    openNotifier();
  }

  public void openNotifier() {
    shuffle.startPeriodic(0.1);
  }

  private void updateShuffleboard() {
    SmartDashboard.putNumber("Wheel Vel", getVelocity());
    SmartDashboard.putNumber("Wheel Volts", leader.get() * RobotController.getBatteryVoltage());
  }

  public void set(double percentage) {
    leader.set(ControlMode.PercentOutput, percentage);
    follower.set(ControlMode.PercentOutput, percentage);
  }

  public void setVoltage(double voltage) {
    set(voltage / RobotController.getBatteryVoltage());
  }

  public void setVelocity(double velocity) {
    this.velocity = velocity;
  }

  public double getVelocity() {
    return (leader.getSelectedSensorVelocity() * 600.0) / (2048.0 * Constants.shooter.GEARING);
  }

  public double getPosition() {
    return (leader.getSelectedSensorPosition()) / (2048.0 * Constants.shooter.GEARING);
  }

  public double getLeaderTemp() {
    return leader.getTemperature();
  }
  public double getFollowerTemp() {
    return follower.getTemperature();
  }

  public void periodic() {
    setVoltage(pid.calculate(getVelocity(), velocity * Constants.shooter.kF));
  }
}

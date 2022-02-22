// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

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
import frc.robot.Constants;

public class Wheel {

  private LinearSystem<N1, N1, N1> shooterPlant = LinearSystemId.identifyVelocitySystem(Constants.shooter.kV, Constants.shooter.kA);
  private KalmanFilter<N1, N1, N1> filter = new KalmanFilter<>(Nat.N1(), Nat.N1(), shooterPlant, VecBuilder.fill(Constants.shooter.SYSTEM_STDEV), VecBuilder.fill(Constants.shooter.ENC_STDEV), 0.2);
  private LinearQuadraticRegulator<N1, N1, N1> regulator = new LinearQuadraticRegulator<>(shooterPlant, VecBuilder.fill(Constants.shooter.QELMS), VecBuilder.fill(Constants.shooter.RELMS), 0.020);
  private LinearSystemLoop<N1, N1, N1> loop = new LinearSystemLoop<>(shooterPlant, regulator, filter, 12.0, 0.020);

  private WPI_TalonFX leader;
  private WPI_TalonFX follower;
  
  private Notifier shuffle = new Notifier(() -> updateShuffleboard());


  private double velocity = 0.0;

  protected Wheel(int leaderID, int followerID) {
    leader = new WPI_TalonFX(leaderID);
    follower = new WPI_TalonFX(followerID);
    follower.follow(leader);
    loop.reset(VecBuilder.fill(getVelocity()));

    leader.setInverted(true);

    leader.setNeutralMode(NeutralMode.Coast);
    leader.setSelectedSensorPosition(0.0);
    leader.configOpenloopRamp(Constants.shooter.RAMP);

    openNotifier();
  }

  public void openNotifier() {
    shuffle.startPeriodic(0.1);
  }

  private void updateShuffleboard() {
    SmartDashboard.putNumber("Leader Temp", getLeaderTemp());
    SmartDashboard.putNumber("Leader Temp", getFollowerTemp());
    SmartDashboard.putNumber("Wheel Vel", getVelocity());
    SmartDashboard.putNumber("Wheel Volts", leader.get() * RobotController.getBatteryVoltage());
  }

  public void set(double percentage) {
    leader.set(percentage);
  }

  public void setVoltage(double voltage) {
    leader.setVoltage(voltage);
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
    loop.reset(VecBuilder.fill(getVelocity() / 60.0));
    loop.setNextR(VecBuilder.fill(velocity / 60.0));
    loop.correct(VecBuilder.fill(getVelocity() / 60.0));
    loop.predict(0.02);
    
    setVoltage(loop.getU(0));
  }
}

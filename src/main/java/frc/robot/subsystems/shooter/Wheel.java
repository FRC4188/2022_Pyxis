// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.utils.motors.CSPMotor;

public class Wheel {

  private CSPMotor leader = Constants.devices.shooterLeader;
  private CSPMotor follower = Constants.devices.shooterFollower;
  private SimpleMotorFeedforward ff =
      new SimpleMotorFeedforward(Constants.shooter.kS, Constants.shooter.kV, Constants.shooter.kA);
  private PIDController pid =
      new PIDController(Constants.shooter.kP, Constants.shooter.kI, Constants.shooter.kD);
  private double velocity = 0.0;

  protected Wheel() {
    leader.reset();
    follower.reset();

    leader.setInverted(false);
    follower.setInverted(true);

    leader.setBrake(false);
    leader.setRamp(0.0);
    follower.setRamp(0.0);
  }

  public void set(double percentage) {
    leader.set(percentage);
    follower.set(percentage);
  }

  public void setVoltage(double voltage) {
    set(voltage / RobotController.getBatteryVoltage());
  }

  public void setVelocity(double velocity) {
    this.velocity = velocity;
  }

  public double getVelocity() {
    return (leader.getVelocity() * 60.0) / Constants.shooter.GEARING;
  }

  public double getPosition() {
    return leader.getPosition() / Constants.shooter.GEARING;
  }

  public double getLeaderTemp() {
    return leader.getTemperature();
  }

  public double getFollowerTemp() {
    return follower.getTemperature();
  }

  public void periodic() {
    setVoltage(
        pid.calculate(getVelocity() / 60.0, velocity / 60.0) + ff.calculate(velocity / 60.0));
  }
}

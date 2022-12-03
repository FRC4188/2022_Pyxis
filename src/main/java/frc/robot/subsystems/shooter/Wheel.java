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
      new SimpleMotorFeedforward(Constants.shooter.kS, Constants.shooter.kV);
  private PIDController pid =
      new PIDController(Constants.shooter.kP, 0, 0);
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

    double measured = getVelocity();
    // double roboDelay = 0.05;
    // double volts = leader.getVoltage();
    // double I = 0.01125;
    // double R = (Constants.shooter.kV *0.410833*6380-12*0.410833)/(6380*Math.PI/30);
    // double Fs = Constants.shooter.kS*0.410833;

    // double asym = (volts*0.410833*6380-Fs*6380)/(0.410833*360/Math.PI+R*6380);
    // double C = 2*Math.PI*measured/60-asym;
    // double prediction = C*Math.pow(Math.exp(1),(-(360/Math.PI)*0.410833*roboDelay/(I*6380)-R*roboDelay/I))+asym;

    // prediction = 60*prediction/(2*Math.PI);

    setVoltage(
        pid.calculate(measured, velocity ) + ff.calculate(velocity));
  }
}

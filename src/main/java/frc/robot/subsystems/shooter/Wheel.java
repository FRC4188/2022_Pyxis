// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
import frc.robot.utils.motors.CSPMotor;

public class Wheel extends SubsystemBase{

  private static Wheel wheel;
  public static synchronized Wheel getInstance() {
    if (wheel == null) wheel = new Wheel();
    return wheel;
  }
  private LinearSystem<N1, N1, N1> shooterPlant = LinearSystemId.identifyVelocitySystem(Constants.FLYWHEEL.kV, Constants.FLYWHEEL.kA);
  private KalmanFilter<N1, N1, N1> filter = new KalmanFilter<>(Nat.N1(), Nat.N1(), shooterPlant, 
    VecBuilder.fill(3.0), 
    VecBuilder.fill(0.01), 
    0.020);
  private LinearQuadraticRegulator<N1, N1, N1> regulator = new LinearQuadraticRegulator<>(shooterPlant, 
    //VecBuilder.fill(80.0), 
    //VecBuilder.fill(12.0),
    // make this really really high
     VecBuilder.fill(46596.2), 
     VecBuilder.fill(7.0),
    0.020);
  private LinearSystemLoop<N1, N1, N1> loop = new LinearSystemLoop<>(shooterPlant, regulator, filter, 12.0, 0.020);

  private CSPMotor leader = Constants.devices.shooterLeader;
  private CSPMotor follower = Constants.devices.shooterFollower;

  private Notifier shuffle = new Notifier(() -> updateShuffleboard());
 //private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(Constants.shooter.kS, Constants.shooter.kV, Constants.shooter.kA);
  //private PIDController pid = new PIDController(Constants.shooter.kP, Constants.shooter.kI, Constants.shooter.kD);


  public Wheel() {
    CommandScheduler.getInstance().registerSubsystem(this);

    leader.reset();
    follower.reset();

    leader.setInverted(false);
    follower.setInverted(true);

    leader.setBrake(false);
    leader.setRamp(0.0);
    follower.setRamp(0.0);

    loop.reset(VecBuilder.fill(getVelocity()));

    SmartDashboard.putNumber("Set Flywheel Velocity", 0.0);
    SmartDashboard.putNumber("Set Percentage", 0.0);
    SmartDashboard.putNumber("Set Voltage", 0.0);
    //SmartDashboard.putNumber("kS", 0.0);
    
    shuffle.startPeriodic(0.1);
  }

  @Override
  public void periodic() {

  }
  
  public void openNotifier() {
    shuffle.startPeriodic(0.1);
  }

  private void updateShuffleboard() {
    SmartDashboard.putNumber("1. Flywheel Velocity (RPM)", getVelocity());
    SmartDashboard.putNumber("2. LQR Reference", getNextReference());
    SmartDashboard.putNumber("3. Control input", getInputVoltage());
    SmartDashboard.putNumber("4. State Estimates", getEstimatedVelocity());

  }

  public void set(double percentage) {
    leader.set(percentage);
    follower.set(percentage);
  }

  public void setVoltage(double voltage) {
    set(voltage / RobotController.getBatteryVoltage());
  }

  public void setVelocity(double velocity) {
    loop.setNextR(VecBuilder.fill(velocity));
    loop.correct(VecBuilder.fill(getVelocity()));
    loop.predict(0.02);
    
    setVoltage(loop.getU(0) + Math.signum(loop.getU(0)) * Constants.FLYWHEEL.kS);
  }

  public double getVelocity() {
    return (leader.getVelocity() * 60.0) / Constants.shooter.GEARING;
  }

  public double getLeaderTemp() {
    return leader.getTemperature();
  }
  public double getFollowerTemp() {
    return follower.getTemperature();
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

  
}

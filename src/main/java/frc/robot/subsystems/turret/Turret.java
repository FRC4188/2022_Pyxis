// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.turret.Hunt;
import frc.robot.commands.turret.TurretAngleWait;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.sensors.Sensors;
import frc.robot.utils.motors.CSPMotor;

public class Turret extends SubsystemBase {
  private static Turret instance;

  public static synchronized Turret getInstance() {
    if (instance == null) instance = new Turret();
    return instance;
  }

  private CSPMotor motor = Constants.devices.turretMotor;

  private PIDController targetPID = new PIDController(Constants.turret.TkP, Constants.turret.TkI, Constants.turret.TkD);
  private PIDController positionPID = new PIDController(Constants.turret.PkP, Constants.turret.PkI, Constants.turret.PkD/*, new Constraints(Constants.turret.MAX_VEL, Constants.turret.MAX_ACCEL)*/);

  Notifier notifier = new Notifier(() -> updateShuffleboard());

  /** Creates a new Turret. */
  public Turret() {
    CommandScheduler.getInstance().registerSubsystem(this);

    initialize();

    startNotifier();
  }

  private void initialize() {
    motor.reset();

    motor.setBrake(false);

    motor.setRamp(0.1);

    motor.set(0.0);

    new Trigger(() -> getPosition() >= Constants.turret.MAX_ANGLE).whenActive(new ParallelDeadlineGroup(
      new TurretAngleWait(getPosition() - 180.0).withTimeout(0.45),
      new RunCommand(() -> setVolts(-12.0))
    ).andThen(new Hunt(true)));

    new Trigger(() -> getPosition() <= Constants.turret.MIN_ANGLE).whenActive(new ParallelDeadlineGroup(
      new TurretAngleWait(getPosition() + 180.0).withTimeout(0.45),
      new RunCommand(() -> setVolts(12.0))
    ).andThen(new Hunt(false)));
  }

  private void startNotifier() {
    notifier.startPeriodic(0.2);
  }

  private void updateShuffleboard() {
    SmartDashboard.putNumber("Turret Motor Temp", getTemperature());
    SmartDashboard.putNumber("Turret Position", getPosition());
  }

  public void set(double percent) {
    if (getPosition() < Constants.turret.MIN_ANGLE && percent < 0.0 || 
      getPosition() > Constants.turret.MAX_ANGLE && percent > 0.0) {
      motor.set(0.0);
    } else {
      motor.set(percent);
    }
  }

  public void setVolts(double volts) {
    set(volts / RobotController.getBatteryVoltage());
  }

  public void setAngle(double angle) {
    setVolts(positionPID.calculate(getPosition(), angle));
  }

  public void trackTarget() {
    if (Sensors.getInstance().getHasTarget()) setVolts(targetPID.calculate(0.0, Sensors.getInstance().getTargetAngle()) + -Swerve.getInstance().getChassisSpeeds().omegaRadiansPerSecond * 2.5);
    else set(0.0);
  }

  public double getPosition() {
    return motor.getPosition() * Constants.turret.ENCODER_TO_DEGREES;
  }

  public double getTemperature() {
    return motor.getTemperature();
  }

  public boolean getIsAimed() {
    double angle = Sensors.getInstance().getTargetAngle();
    boolean aimed = (Math.abs(angle) < Constants.turret.ANGLE_TOLERANCE) && Sensors.getInstance().getHasTarget();
    return aimed;
  }

public double getVelocity() {
  return motor.getVelocity() * Constants.turret.ENCODER_TO_DEGREES;
}
}
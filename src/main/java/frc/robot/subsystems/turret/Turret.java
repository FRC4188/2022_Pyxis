// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import javax.imageio.plugins.tiff.GeoTIFFTagSet;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.sensors.Sensors;

public class Turret extends SubsystemBase {
  private static Turret instance;

  public static synchronized Turret getInstance() {
    if (instance == null) instance = new Turret();
    return instance;
  }

  private CANSparkMax motor = new CANSparkMax(11, MotorType.kBrushless);
  private RelativeEncoder encoder = motor.getEncoder();

  //private ProfiledPIDController pid = new ProfiledPIDController(Constants.turret.kP, Constants.turret.kI, Constants.turret.kD, Constants.turret.CONSTRAINTS);
  private PIDController pid = new PIDController(Constants.turret.kP, Constants.turret.kI, Constants.turret.kD);

  Notifier notifier = new Notifier(() -> updateShuffleboard());

  /** Creates a new Turret. */
  public Turret() {
    CommandScheduler.getInstance().registerSubsystem(this);

    initialize();

    startNotifier();

    putToShuffleboard();
  }

  private void initialize() {
    encoder.setPositionConversionFactor(Constants.turret.ENCODER_TO_DEGREES);
    encoder.setPosition(0.0);

    motor.setIdleMode(IdleMode.kBrake);

    motor.setClosedLoopRampRate(1.0);
    motor.setOpenLoopRampRate(1.0);
  }

  private void startNotifier() {
    notifier.startPeriodic(1.0);
  }

  private void putToShuffleboard() {
    SmartDashboard.putNumber("Set P", 0);
    SmartDashboard.putNumber("Set I", 0);
    SmartDashboard.putNumber("Set D", 0);

    SmartDashboard.putNumber("Set Angle", 0);
  }

  private void updateShuffleboard() {
    SmartDashboard.putNumber("Turret Motor Temp", getTemperature());
    SmartDashboard.putNumber("Turret Position", getPosition());
  }

  public void setPIDs(double kP, double kI, double kD) {
    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
  }

  public void set(double percent) {
    if (getPosition() < Constants.turret.MIN_ANGLE && percent < 0.0 || 
      getPosition() > Constants.turret.MAX_ANGLE && percent > 0.0) {
      motor.set(0.0);
    } else {
      motor.set(percent);
    }
  }

  public void setAngle(double angle) {
    set(12.0 * (pid.calculate(getPosition(), angle)) / RobotController.getBatteryVoltage());
  }

  public void trackTarget(boolean cont) {
    double angle = Sensors.getInstance().getTX();
    double power = 12* 
        pid.calculate(angle, 0.0) / RobotController.getBatteryVoltage()
            + Swerve.getInstance().getChassisSpeeds().omegaRadiansPerSecond / 10.0;

    setAngle(getPosition() + angle);
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public double getTemperature() {
    return motor.getMotorTemperature();
  }

  public boolean getIsAimed() {
    double angle = Sensors.getInstance().getTX();
    boolean aimed = (Math.abs(angle) < Constants.turret.ANGLE_TOLERANCE);
    return aimed;
  }
}
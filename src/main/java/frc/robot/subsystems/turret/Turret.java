/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.turret;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.sensors.Sensors;

/** Turret subsystem class; controls the turret mechanism. */
public class Turret extends SubsystemBase {

  private static Turret instance;

  /**
   * Returns the instance of the {@link Turret} subsystem.
   *
   * @return An instance of {@link Turret} common to the entire program.
   */
  public static synchronized Turret getInstance() {
    if (instance == null) instance = new Turret();
    return instance;
  }

  Sensors sensors = Sensors.getInstance();
  Swerve drive = Swerve.getInstance();

  // Motor control components.
  CANSparkMax turretMotor = new CANSparkMax(42, MotorType.kBrushless);
  RelativeEncoder turretEncoder = turretMotor.getEncoder();
  PIDController pid =
      new PIDController(Constants.turret.kP, Constants.turret.kI, Constants.turret.kD);

  // SmartDashboard thread.
  Notifier shuffle = new Notifier(() -> updateShuffleboard());

  /** Creates a new {@link Turret}. For use only within the {@link Turret} class. */
  private Turret() {
    CommandScheduler.getInstance().registerSubsystem(this);

    motorInits();
    resetEncoders();

    openNotifier();
  }

  @Override
  public void periodic() {}

  /** Configures the turret motor controller. */
  private void motorInits() {
    pid.setP(Constants.turret.kP);
    pid.setI(Constants.turret.kI);
    pid.setD(Constants.turret.kD);

    turretMotor.setClosedLoopRampRate(0.0);
    turretMotor.setOpenLoopRampRate(0.5);

    turretMotor.setIdleMode(IdleMode.kBrake);
  }

  /** Resets turret encoder position value to 0. */
  public void resetEncoders() {
    turretEncoder.setPosition(0.0);
  }

  /** Refreshes the data on SmartDashboard. Should be called in a {@link Notifier}. */
  private void updateShuffleboard() {
    SmartDashboard.putNumber("Turret Angle", getPosition());
  }

  /** End the SmartDashboard interaction notifier. */
  public void closeNotifier() {
    shuffle.close();
  }

  /** Start the SmartDashboard interaction notifier. */
  public void openNotifier() {
    shuffle.startPeriodic(0.1);
  }

  /**
   * Sets turret motor to given percentage [-1.0, 1.0]. Will not allow turret to spin past the
   * software limit
   *
   * @param percent The goal percentage to set the turret motor to.
   */
  public void set(double percent) {
    if (getPosition() < Constants.turret.MIN_ANG && percent < 0.0) turretMotor.set(0.0);
    else if (getPosition() > Constants.turret.MAX_ANG && percent > 0.0) turretMotor.set(0.0);
    else turretMotor.set(percent);
  }

  /**
   * Turns turret to angle in degrees.
   *
   * @param angle Angle for the turret to move to.
   */
  public void setAngle(double angle) {
    angle /= Constants.turret.ENCODER_TO_DEGREES;
    turretMotor.set(Robot.normalizePercentVolts(pid.calculate(turretEncoder.getPosition(), angle)));
  }

  /**
   * Method to track the limelight target
   *
   * @param cont whether to continue tracking or stop.
   */
  public void trackTarget(boolean cont) {
    double angle = sensors.getTX();
    double power =
        Robot.normalizePercentVolts(pid.calculate(angle, 0.0))
            + Swerve.getInstance().getChassisSpeeds().omegaRadiansPerSecond / 10.0;

    set(cont ? power : 0.0);
  }

  /**
   * Returns turret encoder position in degrees.
   *
   * @return Degrees of the turret's current rotation.
   */
  public double getPosition() {
    return turretEncoder.getPosition() * Constants.turret.ENCODER_TO_DEGREES;
  }

  /**
   * Returns turret encoder velocity in degrees per second.
   *
   * @return Velocity of the turret in degrees per second.
   */
  public double getVelocity() {
    return turretEncoder.getVelocity() * Constants.turret.ENCODER_TO_DEGREES / 60.0;
  }

  /**
   * Returns the temperature of the turret motor.
   *
   * @return Temperature (Celsius).
   */
  public double getTemp() {
    return turretMotor.getMotorTemperature();
  }

  /**
   * Method to determine if the turret is aimed at the limelight target.
   *
   * @return Whether the turret is correctly aimed.
   */
  public boolean isAimed() {
    double angle = sensors.getTX();
    boolean aimed = (Math.abs(angle) < Constants.turret.POS_TOLERANCE);
    return aimed;
  }
}

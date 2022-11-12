// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

  private PIDController targetPID =
      new PIDController(Constants.turret.TkP, Constants.turret.TkI, Constants.turret.TkD);
  private PIDController positionPID =
      new PIDController(
          Constants.turret.PkP,
          Constants.turret.PkI,
          Constants.turret
              .PkD /*, new Constraints(Constants.turret.MAX_VEL, Constants.turret.MAX_ACCEL)*/);

  private SimpleMotorFeedforward ff =
      new SimpleMotorFeedforward(Constants.turret.kS, Constants.turret.kV);

  private Swerve swerve = Swerve.getInstance();

  Notifier notifier = new Notifier(() -> updateShuffleboard());

  /** Creates a new Turret. */
  public Turret() {
    CommandScheduler.getInstance().registerSubsystem(this);

    initialize();

    startNotifier();

    SmartDashboard.putNumber("T kP", 0.0);
    SmartDashboard.putNumber("T kI", 0.0);
    SmartDashboard.putNumber("T kD", 0.0);
  }

  private void initialize() {
    motor.reset();

    motor.setBrake(false);

    motor.setRamp(0.1);

    motor.set(0.0);
  }

  private void startNotifier() {
    notifier.startPeriodic(0.2);
  }

  private void updateShuffleboard() {
    // SmartDashboard.putNumber("Turret Motor Temp", getTemperature());
    // SmartDashboard.putNumber("Turret Position", getPosition());
    // double goodAngle =
    //     -(Units.radiansToDegrees(
    //             Math.atan2(
    //                 -Swerve.getInstance().getPose().getY(), -Swerve.getInstance().getPose().getX()))
    //         + Sensors.getInstance().getRotation().getDegrees()
    //         - 180);
    // // if (getPosition() - goodAngle > 90) {
    // //   goodAngle = (goodAngle + 360) % 360 - 180;
    // // }/* else if (getPosition() - goodAngle < -90) {
    // //   goodAngle = (goodAngle - 360) % 360 - 180;
    // // }*/
    // goodAngle = goodAngle % 360.0;
    // SmartDashboard.putNumber("Turret Set", goodAngle);
  }

  public void set(double percent) {
    if (getPosition() < Constants.turret.MIN_ANGLE && percent < 0.0
        || getPosition() > Constants.turret.MAX_ANGLE && percent > 0.0) {
      motor.set(0.0);
    } else {
      motor.set(percent);
    }
  }

  public void setVolts(double volts) {
    set(volts / RobotController.getBatteryVoltage());
  }

  public void setAngle(double angle) {
    positionPID.disableContinuousInput();
    setVolts(positionPID.calculate(getPosition(), angle));
  }

  public void targetAngle(double angle) {
    positionPID.enableContinuousInput(0.0, 360.0);

    double rotSpeed = -swerve.getChassisSpeeds().omegaRadiansPerSecond;
    double velAngle =
        -Math.atan2(
            swerve.getChassisSpeeds().vyMetersPerSecond,
            swerve.getChassisSpeeds().vxMetersPerSecond);
    double totalAngle =
        velAngle
            + Units.degreesToRadians(getPosition())
            + Math.PI / 2
            + Units.degreesToRadians(Sensors.getInstance().getTX());
    double x = Math.sin(totalAngle) * Sensors.getInstance().getDistance();

    setVolts(
        positionPID.calculate(getPosition(), angle)
            + ff.calculate(
                rotSpeed
                    + (x * swerve.getSpeed() / Math.pow(Sensors.getInstance().getDistance(), 2))));
  }

  public void trackTarget() {
    // if (Sensors.getInstance().getHasTarget()) setVolts(targetPID.calculate(0.0,
    // Sensors.getInstance().getTX()
    //   + (Math.abs(Sensors.getInstance().getOffsetAngle()) < 30.0 ?
    // Sensors.getInstance().getOffsetAngle() : 0.0)
    //   + -Swerve.getInstance().getChassisSpeeds().omegaRadiansPerSecond * 4.0));
    // else set(0.0);
    if (Sensors.getInstance().getHasTarget()) {
      double rotSpeed = -swerve.getChassisSpeeds().omegaRadiansPerSecond;
      double velAngle =
          -Math.atan2(
              swerve.getChassisSpeeds().vyMetersPerSecond,
              swerve.getChassisSpeeds().vxMetersPerSecond);
      double angle =
          velAngle
              + Units.degreesToRadians(getPosition())
              + Math.PI / 2
              + Units.degreesToRadians(Sensors.getInstance().getTX());
      double x = Math.sin(angle) * Sensors.getInstance().getDistance();
      setVolts(
          targetPID.calculate(
                  0.0,
                  Sensors.getInstance().getTX()
                      + (Math.abs(Sensors.getInstance().getOffsetAngle()) < 30.0
                          ? Sensors.getInstance().getOffsetAngle()
                          : 0.0))
              + ff.calculate(
                  rotSpeed
                      + (x
                          * swerve.getSpeed()
                          / Math.pow(Sensors.getInstance().getDistance(), 2))));
    } else set(0.0);
  }

  public void setTPID(double kP, double kI, double kD) {
    targetPID.setPID(kP, kI, kD);
  }

  public double getPosition() {
    return motor.getPosition() * Constants.turret.ENCODER_TO_DEGREES;
  }

  public double getTemperature() {
    return motor.getTemperature();
  }

  public boolean getIsAimed() {
    double angle = Sensors.getInstance().getTargetAngle();
    boolean aimed =
        (Math.abs(angle) < Constants.turret.ANGLE_TOLERANCE)
            && Sensors.getInstance().getHasTarget();
    return aimed;
  }

  public double getVelocity() {
    return motor.getVelocity() * Constants.turret.ENCODER_TO_DEGREES;
  }
}

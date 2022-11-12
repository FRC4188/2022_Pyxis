package frc.robot.subsystems.trigger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.motors.CSPMotor;

public class PreShooter extends SubsystemBase {

  private static PreShooter instance = null;

  public static synchronized PreShooter getInstance() {
    if (instance == null) instance = new PreShooter();
    return instance;
  }

  // private Notifier dashboard = new Notifier(() -> updateDashboard());

  private CSPMotor motor = Constants.devices.preshooterMotor;
  ;
  private DigitalInput top = Constants.devices.top;
  private DigitalInput bot = Constants.devices.bot;

  /** Creates a new Trigger. */
  private PreShooter() {
    motor.reset();
    motor.setRamp(0.25);
    // dashboard.startPeriodic(0.35);
  }

  private void updateDashboard() {
    // SmartDashboard.putBoolean("Top Beam Breaker", getTop());
    // SmartDashboard.putBoolean("Bottom Beam Breaker", getBottom());
    // SmartDashboard.putNumber("Trigger Motor Temperature", getTemp());
  }

  public double getTemp() {
    return motor.getTemperature();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double power) {
    motor.set(power);
  }

  public void setVoltage(double volts) {
    set(volts / RobotController.getBatteryVoltage());
  }

  public boolean getTop() {
    return !top.get();
  }

  public boolean getBottom() {
    return !bot.get();
  }
}

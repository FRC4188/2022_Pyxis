package frc.robot.subsystems.trigger;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Trigger extends SubsystemBase {

  private static Trigger instance = null;
  public static synchronized Trigger getInstance() {
    if (instance == null) instance = new Trigger();
    return instance;
  }

  private Notifier dashboard = new Notifier(() -> updateDashboard());

  private WPI_TalonFX motor;

  private DigitalInput top = new DigitalInput(Constants.indexer.TOP_BB);
  private DigitalInput bot = new DigitalInput(Constants.indexer.BOTTOM_BB);

  /** Creates a new Trigger. */
  private Trigger() {
    motor = new WPI_TalonFX(Constants.indexer.TRIGGER_ID);
    motor.configFactoryDefault();
    motor.configOpenloopRamp(0.25);
    dashboard.startPeriodic(0.1);
  }

  private void updateDashboard() {
    SmartDashboard.putBoolean("Top Beam Breaker", getTop());
    SmartDashboard.putBoolean("Bottom Beam Breaker", getBottom());
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

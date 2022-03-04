package frc.robot.subsystems.trigger;

import com.ctre.phoenix.motorcontrol.StatusFrame;
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
    motor = new WPI_TalonFX(Constants.indexer.TRIGGER_ID, "Pyxis CANivore");
    motor.configFactoryDefault();
    motor.configOpenloopRamp(0.25);
    dashboard.startPeriodic(0.1);
    motor.clearStickyFaults();
    motor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    motor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 255);
    motor.setStatusFramePeriod(StatusFrame.Status_10_Targets, 255);
    motor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 255);
    motor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 255);
    motor.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 255);
    motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
    motor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 255);
    motor.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 255);
    motor.setStatusFramePeriod(StatusFrame.Status_6_Misc, 255);


  }

  private void updateDashboard() {
    SmartDashboard.putBoolean("Top Beam Breaker", getTop());
    SmartDashboard.putBoolean("Bottom Beam Breaker", getBottom());
    SmartDashboard.putNumber("Trigger Motor Temperature", getTemp());
  }

  private double getTemp() {
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

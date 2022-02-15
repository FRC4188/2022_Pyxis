package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private static Intake instance = null;
  public static synchronized Intake getInstance() {
    if (instance == null) instance = new Intake();
    return instance;
  }

  TalonFX motor = new TalonFX(Constants.intake.MOTOR_ID);

  /** Creates a new Intake. */
  private Intake() {
    motor.configFactoryDefault();
    motor.configOpenloopRamp(Constants.intake.RAMP_RATE);

    SmartDashboard.putNumber("Intake Voltage", 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double percent) {
    motor.set(ControlMode.PercentOutput, percent);
  }

  public void setVoltage(double power) {
    set(power / RobotController.getBatteryVoltage());
  }
}

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.sensors.Sensors;

public class Shooter extends SubsystemBase {

  private static Shooter instance = null;
  public static synchronized Shooter getInstance() {
    if (instance == null) instance = new Shooter();
    return instance;
  }

  private Notifier dashboard;
  Wheel wheel = new Wheel(Constants.shooter.LEADER_ID, Constants.shooter.FOLLOWER_ID);
  Sensors sensors = Sensors.getInstance();

  /** Creates a new Shooter. */
  private Shooter() {
    CommandScheduler.getInstance().registerSubsystem(this);

    SmartDashboard.putNumber("Shooter Set Velocity", 0.0);
    SmartDashboard.putNumber("Hood Set Angle", 0.0);

    dashboard = new Notifier(() -> updateDashboard());
    dashboard.startPeriodic(0.1);
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("Shooter Velocity", getVelocity());
    SmartDashboard.putNumber("Leader Temp", wheel.getLeaderTemp());
    SmartDashboard.putNumber("Follower Temp", wheel.getFollowerTemp());
    SmartDashboard.putBoolean("Shooter is Ready", isReady());
  }

  public void setVolts(double volts) {
    wheel.setVoltage(volts);
  }

  public void setVelocity(double rpm) {
    wheel.setVelocity(rpm);
  }

  public double getVelocity() {
    return wheel.getVelocity();
  }

  @Override
  public void periodic() {
    wheel.periodic();
  }

public boolean isReady() {
    return Math.abs(sensors.getTX()) < 8.0 && Math.abs(getVelocity()-sensors.getFormulaRPM()) < 200.0;
}
}

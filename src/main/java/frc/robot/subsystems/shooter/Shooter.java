package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.sensors.Sensors;

public class Shooter extends SubsystemBase {

  private static Shooter instance = null;
  public static synchronized Shooter getInstance() {
    if (instance == null) instance = new Shooter();
    return instance;
  }

  Wheel wheel = new Wheel();
  Sensors sensors = Sensors.getInstance();

  /** Creates a new Shooter. */
  private Shooter() {
    CommandScheduler.getInstance().registerSubsystem(this);

    SmartDashboard.putNumber("Shooter Set Velocity", 0.0);
    SmartDashboard.putNumber("Hood Set Angle", 0.0);
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("Shooter Velocity", getVelocity());
    SmartDashboard.putNumber("Leader Temp", wheel.getLeaderTemp());
    SmartDashboard.putNumber("Follower Temp", wheel.getFollowerTemp());
    SmartDashboard.putBoolean("Shooter is ready", isReady());
    SmartDashboard.putBoolean("Limelight Component", Math.abs(sensors.getTX() + sensors.getOffsetAngle()) < 2.5);
    SmartDashboard.putBoolean("RPM Component", Math.abs(getVelocity()-sensors.getFormulaRPM()) < 250.0);
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

  public boolean isReady(double rpm) {
    return Math.tan(Math.toRadians(Math.abs(sensors.getTX() + sensors.getOffsetAngle()))) * sensors.getEffectiveDistance() < 0.25 && Math.abs(getVelocity()-rpm) < 250.0 && sensors.getHasTarget() && ((sensors.getDistance() < 4.0 && Swerve.getInstance().getSpeed() < 1.5) || (sensors.getDistance() < 6.0 && Swerve.getInstance().getSpeed() < 0.01)) && Swerve.getInstance().getAccel() < 2.0;
  }

  public boolean isReady() {
      return isReady(sensors.getFormulaRPM());
  }
}

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private static Shooter instance = null;
  public static synchronized Shooter getInstance() {
    if (instance == null) instance = new Shooter();
    return instance;
  }
  UpperShooter upper = new UpperShooter(Constants.shooter.upper.MOTOR_ID);
  LowerShooter lower = new LowerShooter(Constants.shooter.lower.MOTOR_ID);

  /** Creates a new Shooter. */
  private Shooter() {
    CommandScheduler.getInstance().registerSubsystem(this);

    SmartDashboard.putNumber("Shooter Set Velocity", 0.0);
  }

  public void setVolts(double lowerVolts, double upperVolts) {
    lower.setVoltage(lowerVolts);
    upper.setVoltage(upperVolts);
  }

  public void setVelocity(double rpm) {
    lower.setVelocity(rpm);
    upper.setVelocity(rpm / 2.0);
  }

  public double getVelocity() {
    return lower.getVelocity() / 2.0 + upper.getVelocity() / 4.0;
  }

  @Override
  public void periodic() {
    upper.periodic();
    lower.periodic();
  }
}

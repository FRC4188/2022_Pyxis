package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private static Shooter instance;

  /**
   * Returns the instance of the {@link Shooter} subsystem.
   *
   * @return An instance of {@link Shooter} common to the entire program.
   */
  public static synchronized Shooter getInstance() {
    if (instance == null) instance = new Shooter();
    return instance;
  }

  private ShooterWheel wheel = new ShooterWheel(10, 11);

  private Notifier shuffle = new Notifier(() -> updateShuffleboard());

  public Shooter() {
    SmartDashboard.putNumber("Set Shooter Velocity", 0.0);
    SmartDashboard.putNumber("Set Shooter Power", 0.0);

    CommandScheduler.getInstance().registerSubsystem(this);

    openNotifier();
  }

  @Override
  public void periodic() {}

  private void updateShuffleboard() {
    SmartDashboard.putNumber("Shooter Speed", getVelocity());
    SmartDashboard.putNumber(
        "Shooter Voltage", wheel.getPower() * RobotController.getInputVoltage());

    if (!wheel.matchingVels()) {
      DriverStation.reportError("Shooter motors spinning at different speeds.", false);
    }
  }

  public void closeNotifier() {
    shuffle.close();
  }

  public void openNotifier() {
    shuffle.startPeriodic(0.1);
  }

  /** Sets shooter motors to a given percentage [-1.0, 1.0]. */
  public void setPercentage(double percent) {
    wheel.setPower(percent);
  }

  /** Sets shooter motors to a given velocity in rpm. */
  public void setVelocity(double velocity) {
    wheel.setVelocity(velocity);
  }

  /** Gets left shooter motor velocity in rpm. */
  public double getVelocity() {
    return wheel.getVelocity();
  }

  /** Returns left shooter motor temperature in Celcius. */
  public double getLowerTemp() {
    return wheel.getLowerTemp();
  }

  /** Returns right shooter motor temperature in Celcius. */
  public double getUpperTemp() {
    return wheel.getUpperTemp();
  }
}

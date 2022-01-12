package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants;
import frc.robot.utils.MusicPlayer;

/**
 * Class to control both shooter motors as one in the shooter flywheel system. To be used within the
 * {@link Shooter} class.
 */
public class ShooterWheel {

  /**
   * Secondary follower, or "slave" motor. I know it's not PC, but it's a naming convention, you'll
   * just have to deal with it.
   */
  WPI_TalonFX slaveMotor;
  /**
   * Primary leader, or "master" motor. I know it's not PC, but it's a naming convention, you'll
   * just have to deal with it.
   */
  WPI_TalonFX masterMotor;

  public ShooterWheel(int upperCanID, int LowerCanID) {
    slaveMotor = new WPI_TalonFX(upperCanID);
    masterMotor = new WPI_TalonFX(LowerCanID);

    slaveMotor.configFactoryDefault();
    masterMotor.configFactoryDefault();

    masterMotor.config_kP(0, Constants.shooter.kP);
    masterMotor.config_kI(0, Constants.shooter.kI);
    masterMotor.config_kD(0, Constants.shooter.kD);
    masterMotor.config_kF(0, Constants.shooter.kF);

    masterMotor.setNeutralMode(NeutralMode.Coast);

    masterMotor.configClosedloopRamp(Constants.shooter.RAMP_RATE);
    masterMotor.configOpenloopRamp(Constants.shooter.RAMP_RATE);

    masterMotor.setInverted(true);
    slaveMotor.setInverted(InvertType.FollowMaster);

    masterMotor.configClosedloopRamp(Constants.shooter.RAMP_RATE);

    MusicPlayer.getInstance().addMotor(masterMotor);
    MusicPlayer.getInstance().addMotor(slaveMotor);
  }

  /**
   * Set the percent output power of the shooter.
   *
   * @param power Percent represented by a number in range [-1.0, 1.0].
   */
  public void setPower(double power) {
    masterMotor.set(power);
    slaveMotor.set(ControlMode.Follower, masterMotor.getDeviceID());
  }

  /**
   * Set the velocity of the shooter.
   *
   * @param velocity Velocity (RPM).
   */
  public void setVelocity(double velocity) {
    masterMotor.set(ControlMode.Velocity, velocity * Constants.robot.FALCON_ENCODER_TICKS / 600.0);
    slaveMotor.set(ControlMode.Follower, masterMotor.getDeviceID());
  }

  /**
   * The power currently set to the shooter motor.
   *
   * @return Power as a percent in range [-1.0, 1.0].
   */
  public double getPower() {
    return masterMotor.get();
  }

  /**
   * The current velocity of the shooter.
   *
   * @return Velocity (RPM).
   */
  public double getVelocity() {
    return masterMotor.getSelectedSensorVelocity() * 600.0 / Constants.robot.FALCON_ENCODER_TICKS;
  }

  /**
   * Temperature of the lower motor.
   *
   * @return Temperature (Celsius).
   */
  public double getLowerTemp() {
    return masterMotor.getTemperature();
  }

  /**
   * Temperature of the lower motor.
   *
   * @return Temperature (Celsius).
   */
  public double getUpperTemp() {
    return slaveMotor.getTemperature();
  }

  /**
   * Indicated whether the velocity of the two motors match.
   *
   * @return True if they match, false if they do not.
   */
  public boolean matchingVels() {
    return Math.abs(
            masterMotor.getSelectedSensorVelocity() - slaveMotor.getSelectedSensorVelocity())
        < 5;
  }
}

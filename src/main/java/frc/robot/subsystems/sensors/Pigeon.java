package frc.robot.subsystems.sensors;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.geometry.Rotation2d;

public class Pigeon extends PigeonIMU {
  public Pigeon(int CANid) {
    super(CANid);
    super.configFactoryDefault();
    reset();
  }

  public void reset(double angle) {
    this.setFusedHeading(angle);
  }

  public void reset() {
    reset(0.0);
  }

  public Rotation2d get() {
    return Rotation2d.fromDegrees((super.getFusedHeading() + 180.0) % 360.0 - 180.0);
  }
}

package frc.robot.utils.rotationlib;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Rotation2d;

public class Supplier {

  private LinearInterpolation interpolation;
  private Timer timer = new Timer();

  public Supplier(LinearInterpolation interpolation) {
    this.interpolation = interpolation;
  }

  public void start() {
    timer.reset();
    timer.start();
  }

  public Rotation2d get() {
    return interpolation.sample(timer.get());
  }
}

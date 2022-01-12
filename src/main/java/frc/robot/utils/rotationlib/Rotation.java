package frc.robot.utils.rotationlib;

import edu.wpi.first.math.geometry.Rotation2d;

public class Rotation {

  private Rotation2d rotation;
  private double time;

  public Rotation(Rotation2d rotation, double time) {
    this.rotation = rotation;
    this.time = time;
  }

  public double getTime() {
    return time;
  }

  public Rotation2d getRotation() {
    return rotation;
  }
}

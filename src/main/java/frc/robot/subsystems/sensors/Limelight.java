package frc.robot.subsystems.sensors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.Constants;


public class Limelight {
  private NetworkTable limeTable = null;

  private Pose2d goalPose = new Pose2d();
  //private PowerDistribution pdh = new PowerDistribution(0, ModuleType.kRev);

  /** Enum to control camera mode. */
  public enum CameraMode {
    VISION(0),
    CAMERA(1);

    private final int value;

    CameraMode(int value) {
      this.value = value;
    }

    public int getValue() {
      return this.value;
    }
  }

  /** Enum to control LED mode. */
  public enum LedMode {
    DEFAULT(0),
    OFF(1),
    BLINK(2),
    ON(3);

    private final int value;

    LedMode(int value) {
      this.value = value;
    }

    public int getValue() {
      return this.value;
    }
  }

  public Limelight(String tableName) {
    limeTable = NetworkTableInstance.getDefault().getTable(tableName);
    power(true);
  }

  public void setPipeline(int pipeline) {
    limeTable.getEntry("pipeline").setNumber(pipeline);
  }

  public void setCameraMode(CameraMode mode) {
    limeTable.getEntry("camMode").setNumber(mode.getValue());
  }

  public void setLEDMode(LedMode mode) {
    limeTable.getEntry("ledMode").setNumber(mode.getValue());
  }

  public void setGoalPose(Pose2d pose) {
    goalPose = pose;
  }

  public double getAngle() {
    return Math.atan2(getTY(), getTX());
  }

  public double getHypot() {
    return Math.hypot(getTY(), getTX());
  }

  public double getTX() {
    return limeTable.getEntry("tx").getDouble(0.0);
  }

  public double getTY() {
    return limeTable.getEntry("ty").getDouble(0.0);
  }

  public double getVertical() {
    double r = getHypot();
    return Math.cos(getAngle()) * (r + (0.164*r + 0.102) + Constants.turret.MOUNTING_ANGLE);
  }

  public double getHorizontal() {
    double r = getHypot();
    return -Math.sin(getAngle()) * (r + (0.164*r + 0.102) + Constants.turret.MOUNTING_ANGLE);
  }

  public int getTargetCount() {
    return (int) Math.round(limeTable.getEntry("tv").getDouble(0.0));
  }

  public double[] getCamTran() {
    try {
      return limeTable.getEntry("camtran").getDoubleArray(new double[0]);
    } catch (Exception e) {
      // e.printStackTrace();
      return new double[0];
    }
  }

  public Pose2d getPose2d() {
    try {
      double[] dimensions = getCamTran();
      Pose2d goalRel =
          new Pose2d(
              Units.inchesToMeters(dimensions[0]),
              Units.inchesToMeters(dimensions[2]),
              new Rotation2d(Math.toRadians(dimensions[4])));

      double dist = Math.hypot(goalRel.getX(), goalRel.getY());
      double totalAngle =
          Math.atan2(goalRel.getY(), goalRel.getX()) + goalPose.getRotation().getRadians();

      double botX = Math.cos(totalAngle) * dist + goalPose.getX();
      double botY = Math.sin(totalAngle) * dist + goalPose.getY();

      return new Pose2d(
          botX,
          botY,
          new Rotation2d(goalPose.getRotation().getRadians() + goalRel.getRotation().getRadians()));
    } catch (Exception e) {
      // e.printStackTrace();
      return null;
    }
  }

  public CameraMode getCameraMode() throws Exception {
    int value = (int) Math.round(limeTable.getEntry("camMode").getDouble(-1.0));

    switch (value) {
      case 0:
        return CameraMode.VISION;
      case 1:
        return CameraMode.CAMERA;
      default:
        {
          throw new Exception("Invalid Camera Mode Index: " + Integer.toString(value));
        }
    }
  }

  public LedMode getLEDMode() throws Exception {
    int value = (int) Math.round(limeTable.getEntry("ledMode").getDouble(-1.0));

    switch (value) {
      case 0:
        return LedMode.DEFAULT;
      case 1:
        return LedMode.OFF;
      case 2:
        return LedMode.BLINK;
      case 3:
        return LedMode.ON;
      default:
        {
          throw new Exception("Invalid LED Mode Index: " + Integer.toString(value));
        }
    }
  }

  public int getPipeline() {
    return (int) Math.round(limeTable.getEntry("pipeline").getDouble(-1.0));
  }

  public void power(boolean on) {
    //pdh.setSwitchableChannel(on);
  }
}
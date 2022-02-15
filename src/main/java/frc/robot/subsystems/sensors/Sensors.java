package frc.robot.subsystems.sensors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.sensors.Limelight.CameraMode;
import frc.robot.subsystems.sensors.Limelight.LedMode;

public class Sensors extends SubsystemBase {

  private static Sensors instance = null;

  public static synchronized Sensors getInstance() {
    if (instance == null) instance = new Sensors();
    return instance;
  }

  private Limelight limelight = new Limelight("limelight-swervex");
  private Pigeon pigeon = new Pigeon(31);

  private Notifier notifier = new Notifier(() -> updateShuffleboard());

  /** Creates a new Sensors. */
  private Sensors() {
    CommandScheduler.getInstance().registerSubsystem(this);
    
    startNotifier();
  }

  private void updateShuffleboard() {
    SmartDashboard.putNumber("Limelight Distance", getDistance());
    SmartDashboard.putNumber("Pigeon Angle", pigeon.get().getDegrees());
    SmartDashboard.putNumber("Pigeon Compass", pigeon.getCompass().getDegrees());
  }

  private void startNotifier() {
    notifier.startPeriodic(0.4);
  }

  public boolean getHasTarget() {
    return limelight.getTargetCount() > 0;
  }

  public double getTX() {
    return limelight.getHorizontal();
  }

  public double getTY() {
    return limelight.getVertical();
  }

  public double getDistance() {
    return (Constants.field.GOAL_HEIGHT - Constants.turret.LIMELIGHT_HEIGHT)
        / Math.tan(Math.toRadians(getTY() + Constants.turret.MOUNTING_ANGLE));
  }

  public void setPigeonAngle(double angle) {
    pigeon.set(angle);
  }

  public Rotation2d getRotation() {
    return pigeon.get();
  }

  public Pose2d getPosition() {
    try {
      return new Pose2d(limelight.getPose2d().getTranslation(), getRotation());
    } catch (Exception e) {
      return new Pose2d();
    }
  }

  public double getFormulaRPM() {
    double distance = getDistance();
    return 125.561*distance*distance - 529.004*distance + 2807.27;
  }

  public void setLEDMode(LedMode mode) {
    limelight.setLEDMode(mode);
  }

  public void setCameraMode(CameraMode mode) {
    limelight.setCameraMode(mode);
  }

  public void setPipeline(int index) {
    limelight.setPipeline(index);
  }

  public CameraMode getCameraMode() throws Exception {
    return limelight.getCameraMode();
  }

  public LedMode getLEDMode() throws Exception {
    return limelight.getLEDMode();
  }

  public int getPipeline() {
    return limelight.getPipeline();
  }

}
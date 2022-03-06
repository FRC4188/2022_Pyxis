package frc.robot.subsystems.sensors;

import java.util.Arrays;

import edu.wpi.first.math.filter.LinearFilter;
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
  private Pigeon pigeon = new Pigeon(30);

  private Notifier notifier = new Notifier(() -> updateShuffleboard());

  private LinearFilter txFilter = LinearFilter.movingAverage(5);
  double tx = 0.0;

  /** Creates a new Sensors. */
  private Sensors() {
    CommandScheduler.getInstance().registerSubsystem(this);
    
    startNotifier();
  }

  @Override
  public void periodic() {
    tx = limelight.getHorizontal();
  }

  private void updateShuffleboard() {
    SmartDashboard.putNumber("Limelight Distance", getDistance());
    SmartDashboard.putNumber("Pigeon Angle", pigeon.get().getDegrees());
    SmartDashboard.putNumber("Pigeon Compass", pigeon.getCompass().getDegrees());
    SmartDashboard.putString("Pigeon YPR", Arrays.toString(new double[] {pigeon.getYaw(), pigeon.getPitch(), pigeon.getRoll()}));
  }

  public void setLED(boolean on) {
    limelight.setLEDMode(on ? LedMode.ON : LedMode.OFF);
  }

  private void startNotifier() {
    notifier.startPeriodic(0.4);
  }

  public boolean getHasTarget() {
    return limelight.getTargetCount() > 0;
  }

  public double getPitch() {
    return pigeon.getRoll();
  }

  public double getTX() {
    return tx;
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
    return Constants.shooter.ALPHA * 362.0 * distance + 1800.0;
  }

  public double getFormulaAngle() {
    double distance = getDistance();
    return Constants.shooter.hood.ALPHA * 7.18 * distance + 4.29;
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
package frc.robot.subsystems.sensors;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.sensors.Limelight.CameraMode;
import frc.robot.subsystems.sensors.Limelight.LedMode;
import frc.robot.subsystems.turret.Turret;


public class Sensors extends SubsystemBase {

  private static Sensors instance = null;

  public static synchronized Sensors getInstance() {
    if (instance == null) instance = new Sensors();
    return instance;
  }

  private Limelight limelight = Constants.devices.limelight;
  private Pigeon pigeon = Constants.devices.pigeon;
  private BallDetector ballDetector = Constants.devices.ballDetector;
  private ColorSensor colorSensor = Constants.devices.colorSensor;

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
    SmartDashboard.putString("Pigeon YPR", Arrays.toString(new double[] {pigeon.getYaw(), pigeon.getPitch(), pigeon.getRoll()}));
    SmartDashboard.putBoolean("Right Color", isRightColor());
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
    return limelight.getHorizontal();
  }

  public double getTargetAngle() {
    Translation2d tVel = getTargetVelocityVector();
    double tX = -getTimeOfFlight() * tVel.getX();
    double tY = getDistance() - getTimeOfFlight() * tVel.getY();

    return getTX() - Math.toDegrees(Math.atan2(tY, tX)) % 180.0 - 90.0;
  }

  public double getTY() {
    return limelight.getVertical();
  }

  public double getDistance() {
    return (Constants.field.GOAL_HEIGHT - Constants.turret.LIMELIGHT_HEIGHT)
        / Math.tan(Math.toRadians(getTY() + Constants.turret.MOUNTING_ANGLE));
  }

  public double getTargetDistance() {
    Translation2d tVel = getTargetVelocityVector();
    double tX = -getTimeOfFlight() * tVel.getX();
    double tY = getDistance() - getTimeOfFlight() * tVel.getY();

    return Math.hypot(tX, tY);
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

  public double getTimeOfFlight() {
    return -0.0408107 * Math.pow(getDistance(), 2.0) + 0.327671 * getDistance() + 0.292099;
  }

  private Translation2d getTargetVelocityVector() {
    ChassisSpeeds cSpeeds = Swerve.getInstance().getChassisSpeeds();
    double tAngle = Turret.getInstance().getPosition();
    double lAngle = getTX();

    double cAngle = -tAngle+lAngle;

    return new Translation2d(cSpeeds.vxMetersPerSecond, cSpeeds.vyMetersPerSecond).rotateBy(Rotation2d.fromDegrees(-cAngle));
  }

  public double getFormulaRPM() {
    double distance = getTargetDistance();
    return (isRightColor()) ? Constants.shooter.ALPHA * 362.0 * distance + 1800.0 : 1500;
  }

  public double getFormulaAngle() {
    double distance = getTargetDistance();
    return (isRightColor()) ? Constants.shooter.hood.ALPHA * 7.18 * distance + 4.29 : 2.0;
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

  public double[] getClosestInfo() {
    return ballDetector.getClosestInfo(DriverStation.getAlliance());
  }

  public double getClosestTX() {
    return ballDetector.xPixelToAngle(getClosestInfo()[0]);
  }

  public boolean isRightColor() {
    return (DriverStation.getAlliance() == DriverStation.Alliance.Red ? colorSensor.getColor() != 1 : colorSensor.getColor() != -1);
  }

}
package frc.robot.subsystems.sensors;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.sensors.Limelight.CameraMode;
import frc.robot.subsystems.sensors.Limelight.LedMode;
import frc.robot.subsystems.shooter.Shooter;
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

  private SendableChooser<String> alliance = new SendableChooser<>();

  /** Creates a new Sensors. */
  private Sensors() {
    CommandScheduler.getInstance().registerSubsystem(this);
    
    startNotifier();

    alliance.setDefaultOption("FMS", "FMS");
    alliance.addOption("Blue", "Blue");
    alliance.addOption("Red", "Red");

    SmartDashboard.putData("Alliance Color", alliance);
  }

  private void updateShuffleboard() {
    SmartDashboard.putNumber("Limelight Distance", getDistance());
    SmartDashboard.putNumber("Pigeon Angle", pigeon.get().getDegrees());
    SmartDashboard.putNumber("Formula RPM", getFormulaRPM());
    SmartDashboard.putNumber("Pitch", getPitch());
    SmartDashboard.putNumber("Roll", getRoll());
    SmartDashboard.putString("Target Vel Vector", getTargetVelocityVector().toString());
  }

  public void setLED(boolean on) {
    limelight.setLEDMode(on ? LedMode.ON : LedMode.OFF);
  }

  private void startNotifier() {
    notifier.startPeriodic(0.1);
  }

  public boolean getHasTarget() {
    return limelight.getTargetCount() > 0;
  }

  public double getPitch() {
    return pigeon.getRoll();
  }

  public double getRoll() {
    return pigeon.getPitch();
  }

  public double getTX() {
    return limelight.getTX();
  }
  public double getTargetAngle() {
    double tX = Math.sqrt(getDistance()) * -0.3 * getTargetVelocityVector().getX();
    double tY = Math.sqrt(getDistance()) * -0.3 * getTargetVelocityVector().getY();
    double adjust = Math.toDegrees(Math.atan2(tY, getDistance() + tX));

    SmartDashboard.putNumber("Angle Adjust Val", adjust);

    return getTX() + adjust;
  }

  public double getTY() {
    return limelight.getTY();
  }

  public double getDistance() {
    return (Constants.field.GOAL_HEIGHT - Constants.turret.LIMELIGHT_HEIGHT)
        / Math.tan(Math.toRadians(getTY() + Constants.turret.MOUNTING_ANGLE));
  }

  /*
  public double getTargetDistance() {
    Translation2d tVel = getTargetVelocityVector();
    double tX = -getTimeOfFlight() * tVel.getX();
    double tY = getDistance() - getTimeOfFlight() * tVel.getY();

    return Math.hypot(tX, tY);
  }
*/
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

  /*
  public double getTimeOfFlight() {
    double distance = getDistance();
    return -0.0408107 * Math.pow(distance, 2.0) + 0.327671 * distance + 0.292099;
    //return 0.00167838 * Math.pow(distance, 3.0) - 0.0141796 * Math.pow(distance, 2.0) + 0.0606332 * distance + 1.07604;
  }*/

  private Translation2d getTargetVelocityVector() {
    ChassisSpeeds cSpeeds = Swerve.getInstance().getChassisSpeeds();
    double tAngle = Turret.getInstance().getPosition() + 180.0;
    double lAngle = getTX();

    double cAngle = lAngle-tAngle;

    return new Translation2d(cSpeeds.vxMetersPerSecond, -cSpeeds.vyMetersPerSecond).rotateBy(Rotation2d.fromDegrees(cAngle));
  }

  public double getFormulaRPM() {
    double distance = getDistance() + -0.9 * getTargetVelocityVector().getX();
    return (isRightColor() && getDistance() < 5.5) ? Constants.shooter.ALPHA * 362.0 * distance + 1800.0 : 1500;
    //return isRightColor() ? 1787.69 * Math.pow(0.249199, 1.53841 * distance + 0.199996) + 216.565 * distance + 2046.46 : 1500.0;
  }

  public double getFormulaAngle() {
    double distance = getDistance() + -0.9 * getTargetVelocityVector().getX();
    return (isRightColor()) ? Constants.shooter.hood.ALPHA * 7.18 * distance + 4.29 : 2.0;
    //return isRightColor() ? -50.7709 * Math.pow(2.1949, -0.786134 * distance - 0.777002) + 1.71126 * distance + 21.2811 : 2.0;
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
    String alliance = this.alliance.getSelected();
    if (alliance.equals("Red"))
      return colorSensor.getColor() != 1;
    else if (alliance.equals("Blue"))
      return colorSensor.getColor() != -1;
    else return (DriverStation.getAlliance() == DriverStation.Alliance.Red ? colorSensor.getColor() != 1 : colorSensor.getColor() != -1);
  }

  public Translation2d getAdjustedMagnitude() {
    Swerve drive = Swerve.getInstance();
    Turret turret = Turret.getInstance();

    Translation2d driveVector = new Translation2d(drive.getChassisSpeeds().vxMetersPerSecond, drive.getChassisSpeeds().vyMetersPerSecond);
    Translation2d resultxVector = new Translation2d((getFormulaRPM() * 0.1016 * Math.PI * 0.575) / 60 * Math.sin(Math.toRadians(getFormulaAngle() + 8.6)), Rotation2d.fromDegrees(turret.getPosition() - getTX()));
    Translation2d shooterxVector = resultxVector.minus(driveVector);

    return new Translation2d(shooterxVector.getNorm(), Math.atan2(shooterxVector.getY(), shooterxVector.getX()));
  }

  public double getAdjustedRPM() {
    Translation2d adjustedMagnitude = getAdjustedMagnitude();
    double rpmMagnitude = Math.hypot(adjustedMagnitude.getNorm(), (getFormulaRPM() * 0.1016 * Math.PI * 0.575) / 60 * Math.cos(Math.toRadians(getFormulaAngle() + 8.6)));
    rpmMagnitude *= 60.0;
    rpmMagnitude /= 0.1016 * Math.PI * 0.575;
    return rpmMagnitude;
  }

  public double getOffsetAngle() {
    return Math.toDegrees(Math.atan2(getAdjustedMagnitude().getY(), getAdjustedMagnitude().getX()));
  }
}
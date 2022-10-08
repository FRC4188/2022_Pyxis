package frc.robot.subsystems.sensors;

import java.util.Arrays;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.estimator.UnscentedKalmanFilter;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Odometry;
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

  private SendableChooser<String> alliance = new SendableChooser<>();

  private LinearFilter dFilter = LinearFilter.singlePoleIIR(0.05, 0.02);

  private Debouncer zoneFilter = new Debouncer(0.15, DebounceType.kBoth);

  /** Creates a new Sensors. */
  private Sensors() {
    CommandScheduler.getInstance().registerSubsystem(this);
    
    alliance.setDefaultOption("FMS", "FMS");
    alliance.addOption("Blue", "Blue");
    alliance.addOption("Red", "Red");
    alliance.addOption("All", "All");

    SmartDashboard.putData("Alliance Color", alliance);
    

    //setPower(true);
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("Distance to Goal", getDistance());
    SmartDashboard.putNumber("Yaw", pigeon.get().getDegrees());
    SmartDashboard.putNumber("Pitch", getPitch());
    SmartDashboard.putNumber("Roll", getRoll());
    SmartDashboard.putString("Target Vel Vector", getTargetVelocityVector().toString());
    SmartDashboard.putNumber("LL TX", getTX());
    SmartDashboard.putNumber("OTF Angle Adjustment", getOffsetAngle());
    SmartDashboard.putNumber("Closest Ball Angle", getClosestBallAngle());
    SmartDashboard.putNumber("Formula RPM", getFormulaRPM());
    SmartDashboard.putNumber("Formula Angle", getFormulaAngle());

  }

  public void setLED(boolean on) {
    //limelight.setLEDMode(on ? LedMode.ON : LedMode.OFF);
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
    return limelight.getTX() - 1.5;
  }

  public double getTY() {
    return limelight.getTY();
  }

  public double getDistance() {
    return dFilter.calculate((Constants.field.GOAL_HEIGHT - Constants.turret.LIMELIGHT_HEIGHT)
        / Math.tan(Math.toRadians(getTY() + Constants.turret.MOUNTING_ANGLE)));
  }

  public void setPigeonAngle(double angle) {
    pigeon.set(angle);
  }

  public Rotation2d getRotation() {
    return pigeon.get();
  }

  public double getEffectiveDistance() {
    double distance = getDistance() -(getDistance() * 0.1175 + 0.13 + 0.4) * getTargetVelocityVector().getX();
    //distance = Math.hypot(distance, -(getDistance() * 0.12 + 0.15 /*0.15 + 0.4*/) * getTargetVelocityVector().getY());
    return distance;
  }

  public double getFormulaRPM() {
    // double distance = getDistance() + -0.9 * getTargetVelocityVector().getX();
    //double distance = getDistance() -(getDistance() * 0.15 + 0.35) * getTargetVelocityVector().getX();
    double distance = getEffectiveDistance();
    //double rpm = zoneFilter.calculate(distance > 2.2) ? Constants.shooter.ALPHA * 372.0 * distance + 1700.0 : 414.961 * distance +1811.23;
    double rpm = zoneFilter.calculate(distance > 2.2) ? 381.718 * distance + 1924.21 : 414.961 * distance +1811.23;
    return (isRightColor() && getDistance() < 6.5) ? rpm : 2000;
  }

  public double getFormulaAngle() {
    // double distance = getDistance() + -0.9 * getTargetVelocityVector().getX();
    //double distance = getDistance() -(getDistance() * 0.15 + 0.35) * getTargetVelocityVector().getX();
  double distance = getEffectiveDistance();
  double angle = zoneFilter.calculate(distance > 2.2) ?  3.73737 * distance + 9.8277 : 12.0 * distance - 5.57799;
    return (isRightColor()) ? angle : 2.0;
  }

  public void setLEDMode(LedMode mode) {
    //limelight.setLEDMode(mode);
  }

  public void setCameraMode(CameraMode mode) {
    limelight.setCameraMode(mode);
  }

  public void setPipeline(int index) {
    limelight.setPipeline(index);
  }

  public void setPower(boolean power) {
    limelight.power(power);
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

  public boolean isRightColor() {
    String alliance = this.alliance.getSelected();
    if (alliance.equals("Red"))
      return colorSensor.getColor() != 1;
    else if (alliance.equals("Blue"))
      return colorSensor.getColor() != -1;
    else if (alliance.equals("All"))
      return true;
    else return (DriverStation.getAlliance() == DriverStation.Alliance.Red ? colorSensor.getColor() != 1 : colorSensor.getColor() != -1);
  }

  public double getClosestBallAngle() {
    double cameraFOV = 29.4;
    double frameWidth = 640;
    double centerXtoAngle = cameraFOV / (frameWidth / 2);

    double biggestRedAngle = ballDetector.getCenterX((int) ballDetector.getClosestsIndexes()[0][0]) * centerXtoAngle;
    double biggestBlueAngle = ballDetector.getCenterX((int) ballDetector.getClosestsIndexes()[1][0]) * centerXtoAngle;

    String alliance = this.alliance.getSelected();
    if (alliance.equals("Red")) {
      return biggestRedAngle;
    } else if (alliance.equals("Blue")) {
      return biggestBlueAngle;
    } else if (alliance.equals("All")) {
      double biggestRedSize = ballDetector.getClosestsIndexes()[0][1];
      double biggestBlueSize = ballDetector.getClosestsIndexes()[1][1];
      return (biggestRedSize > biggestBlueSize) ?  biggestRedAngle : biggestBlueAngle;
    } else {
      return (DriverStation.getAlliance() == DriverStation.Alliance.Red) ? biggestRedAngle : biggestBlueAngle;
    }
  }

  private Translation2d getTargetVelocityVector() {
    ChassisSpeeds cSpeeds = Swerve.getInstance().getChassisSpeeds();
    double tAngle = Turret.getInstance().getPosition() + 180.0;
    double lAngle = getTX();

    double cAngle = lAngle-tAngle;

    return new Translation2d(cSpeeds.vxMetersPerSecond, -cSpeeds.vyMetersPerSecond).rotateBy(Rotation2d.fromDegrees(cAngle));
  }

  public double getOffsetAngle() {
    // double tX = Math.sqrt(getDistance()) * -0.3 * getTargetVelocityVector().getX();
    // double tY = Math.sqrt(getDistance()) * -0.3 * getTargetVelocityVector().getY();
    double tX = -(getDistance() * 0.13 + 0.2) * getTargetVelocityVector().getX();
    double tY = -(getDistance() * 0.13 + 0.2) * getTargetVelocityVector().getY();
    return Math.toDegrees(Math.atan2(tY, getDistance() + tX));
  }

  public double getTargetAngle() {
    Pose2d goalPose = Swerve.getInstance().getPose();
    return Math.toDegrees(Math.atan2(goalPose.getY(), goalPose.getX())) + getOffsetAngle();
  }

  public Pose2d getVisionPose() {
    double pAngle = getRotation().getRadians();
    double tAngle = Math.toRadians(-Turret.getInstance().getPosition() - 180.0);
    double lAngle = Math.toRadians(getTX());

    double targetToZero = tAngle + pAngle + lAngle;

    double dist = getDistance() + Units.feetToMeters(2.0);
    double x = -Math.cos(targetToZero) * dist;
    double y = -Math.sin(targetToZero) * dist;

    return new Pose2d(x, y, getRotation());
  }
}
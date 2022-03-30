package frc.robot.subsystems.sensors;

import java.util.Arrays;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.estimator.UnscentedKalmanFilter;
import edu.wpi.first.math.filter.LinearFilter;
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

  private Notifier notifier = new Notifier(() -> updateShuffleboard());

  private SendableChooser<String> alliance = new SendableChooser<>();

  private Odometry goalPoseEstimator = new Odometry(new Pose2d());

  private LinearFilter txFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

  /** Creates a new Sensors. */
  private Sensors() {
    CommandScheduler.getInstance().registerSubsystem(this);
    
    startNotifier();

    alliance.setDefaultOption("FMS", "FMS");
    alliance.addOption("Blue", "Blue");
    alliance.addOption("Red", "Red");
    alliance.addOption("All", "All");

    SmartDashboard.putData("Alliance Color", alliance);

    new Trigger(() -> getHasTarget())
      .whileActiveContinuous(new InstantCommand(() -> setGoalPose(calculateGoalPose())))
      .whenInactive(new RunCommand(() -> updateGoalPose()).withInterrupt(() -> getHasTarget()));
  }

  private void updateShuffleboard() {
    SmartDashboard.putNumber("Distance to Goal", getDistance());
    SmartDashboard.putNumber("Yaw", pigeon.get().getDegrees());
    SmartDashboard.putNumber("Pitch", getPitch());
    SmartDashboard.putNumber("Roll", getRoll());
    SmartDashboard.putString("Target Vel Vector", getTargetVelocityVector().toString());
    SmartDashboard.putString("Hub Position", getGoalPose().toString());
    SmartDashboard.putNumber("LL TX", getTX());
    SmartDashboard.putNumber("OTF Angle Adjustment", getOffsetAngle());
  }

  public void setLED(boolean on) {
    limelight.setLEDMode(on ? LedMode.ON : LedMode.OFF);
  }

  private void startNotifier() {
    notifier.startPeriodic(0.2);
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
    return txFilter.calculate(limelight.getTX());
  }

  public double getTY() {
    return limelight.getTY();
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

  public double getFormulaRPM() {
    double distance = getDistance() + -1.15 * getTargetVelocityVector().getX();
    return (isRightColor() && getDistance() < 5.5) ? Constants.shooter.ALPHA * 362.0 * distance + 1800.0 : 1500;
  }

  public double getFormulaAngle() {
    double distance = getDistance() + -1.15 * getTargetVelocityVector().getX();
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
    String alliance = this.alliance.getSelected();
    if (alliance.equals("Red"))
      return colorSensor.getColor() != 1;
    else if (alliance.equals("Blue"))
      return colorSensor.getColor() != -1;
    else if (alliance.equals("All"))
      return true;
    else return (DriverStation.getAlliance() == DriverStation.Alliance.Red ? colorSensor.getColor() != 1 : colorSensor.getColor() != -1);
  }

  private Translation2d getTargetVelocityVector() {
    ChassisSpeeds cSpeeds = Swerve.getInstance().getChassisSpeeds();
    double tAngle = Turret.getInstance().getPosition() + 180.0;
    double lAngle = getTX();

    double cAngle = lAngle-tAngle;

    return new Translation2d(cSpeeds.vxMetersPerSecond, -cSpeeds.vyMetersPerSecond).rotateBy(Rotation2d.fromDegrees(cAngle));
  }

  public double getOffsetAngle() {
    double tX = -1.2 * getTargetVelocityVector().getX();
    double tY = -1.2 * getTargetVelocityVector().getY();
    return Math.toDegrees(Math.atan2(tY, getDistance() + tX));
  }

  public double getTargetAngle() {
    Translation2d goalPose = getGoalPose();
    return Math.toDegrees(Math.atan2(goalPose.getY(), goalPose.getX())) + getOffsetAngle();
  }

  public void setGoalPose(Translation2d pose) {
    goalPoseEstimator.setPose(new Pose2d(pose, new Rotation2d()));
  }

  public Translation2d getGoalPose() {
    return goalPoseEstimator.getPose().getTranslation();
  }

  public void updateGoalPose() {
    ChassisSpeeds speeds = Swerve.getInstance().getChassisSpeeds();
    double rps = speeds.omegaRadiansPerSecond / (2.0 * Math.PI);
    double circumference = getDistance() * 2.0 * Math.PI;
    double rotVel = circumference * -rps;

    Translation2d prevPose = getGoalPose();

    goalPoseEstimator.update(new ChassisSpeeds(
      -speeds.vxMetersPerSecond + Math.sin(Math.atan2(prevPose.getY(), prevPose.getX())) * rotVel,
      -speeds.vyMetersPerSecond + Math.cos(Math.atan2(prevPose.getY(), prevPose.getX())) * rotVel,
      0.0
    ), new Rotation2d());
  }

  public Translation2d calculateGoalPose() {
    double tAngle = -Turret.getInstance().getPosition() - 180.0;
    double lAngle = getTX();
    double angle = tAngle + lAngle;
    double distance = getDistance();

    return new Translation2d(Math.cos(Math.toRadians(angle)) * distance, Math.sin(Math.toRadians(angle)) * distance);
  }
}
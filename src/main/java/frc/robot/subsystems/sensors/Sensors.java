// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import frc.robot.subsystems.sensors.Limelight;

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

    openNotifier();
    
  }

  @Override
  public void periodic() {
  }

  private void openNotifier()
  {
    notifier.startPeriodic(2);
  }

  public boolean hasTarget() {
    return limelight.targetCount() > 0;
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

  public double getFormulaRPM() {
    double dist = getDistance();

    return 37.0583 * Math.pow(dist, 2) + -368.372 * dist + 4566.37;
  }

  private void updateShuffleboard()
  {
    SmartDashboard.putNumber("TX", getTX());
    SmartDashboard.putNumber("TY", getTY());
    SmartDashboard.putBoolean("On Target", hasTarget());
  }
  
  public Rotation2d getRotation() {
    return pigeon.get();
  }


  
  public Pose2d getRobotPose() {
    try {
      return new Pose2d(limelight.getPose2d().getTranslation(), getRotation());
    } catch (Exception e) {
      return new Pose2d();
    }
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

  public void setRotation(Rotation2d newRot) {

  }
}

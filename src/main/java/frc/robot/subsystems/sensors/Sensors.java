// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensors;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Sensors extends SubsystemBase {
  private static Sensors instance;
  public static synchronized Sensors getInstance() {
    if (instance == null) instance = new Sensors();
    return instance;
  }

  private Limelight limelight = new Limelight("limelight");
  private BallDetector detector = new BallDetector("ball detector");
  private ColorSensor sensor = new ColorSensor();
  private Pigeon imu = Constants.Sensors.Devices.IMU;

  private SendableChooser<String> allianceSelector = new SendableChooser<>();
  
  private LinearFilter filter = LinearFilter.singlePoleIIR(0.05, 0.02);
  private Debouncer debouncer = new Debouncer(0.15, DebounceType.kBoth);

  /** Creates a new Sensors. */
  private Sensors() {
    init();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    sensor.refresh();
  }

  private void init() {
    allianceSelector.setDefaultOption("FMS", "FMS");
    allianceSelector.addOption("Blue", "Blue");
    allianceSelector.addOption("Red", "Red");
    allianceSelector.addOption("All", "All");

    SmartDashboard.putData("Alliance Selector", allianceSelector);
  }

  public double[] getClosestBall() {
    String selection = allianceSelector.getSelected();

    return detector.getClosestBall(selection);
  }

  public boolean isCorrectColor() {
    String selection = allianceSelector.getSelected();

    switch (selection) {
      case "Blue":
        return sensor.getColor() == "Blue";
      case "Red":
        return sensor.getColor() == "Red";
      case "All":
        return true; 
      default:
        return (sensor.getColor() == DriverStation.getAlliance().name());
    }
  }

  public double getTY() {
    return limelight.getTY();
  }

  public double getTX() {
    return limelight.getTX() - 1.5;
  }

  public boolean hasTarget() {
    return limelight.hasTarget();
  } 

  public double getDistance() {
    
    return filter.calculate((Constants.Field.GOAL_HEIGHT - Constants.Turret.LL_HEIGHT) / 
            Math.tan(Units.degreesToRadians((getTY() + Constants.Turret.LL_MOUNTING_ANGLE))));
  }

  public Rotation2d getHeading() {
    return imu.getHeading();
  }

  public double getPitch() {
    return imu.getPitch();
  }

  public double getRoll() {
    return imu.getRoll();
  }

  public Pose2d getEstimatedPose() {
    double pigeonAngle = getHeading().getRadians();
    double turretAngle = 0.0;
    double limelightAngle = Units.degreesToRadians(getTX());

    double toZero = pigeonAngle + turretAngle + limelightAngle;

    double distance = getDistance() + Units.feetToMeters(2.0);

    double x = -Math.cos(toZero) * distance;
    double y = -Math.sin(toZero) * distance;

    return new Pose2d(x, y, new Rotation2d(pigeonAngle));
  }

  public double getCalculatedRPM() {
    return 0.0;
  }
}

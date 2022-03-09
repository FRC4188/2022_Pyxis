// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.motors.CSPMotor;

public class Indexer extends SubsystemBase {
  private static Indexer instance;

  public static synchronized Indexer getInstance () {
    if (instance == null) instance = new Indexer();
    return instance;
  }
 private CSPMotor indexer = Constants.devices.indexerMotor;

  /** Creates a new Indexer. */
  private Indexer() {
    indexer.reset();
    indexer.setRamp(0.25);
  }

  @Override
  public void periodic() {
  
  }
  
  public void set(double power){
    indexer.set(power);
  }

  public void setVoltage(double number) {
    set(number / RobotController.getBatteryVoltage());
  }
}

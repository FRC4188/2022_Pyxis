// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  private static Indexer instance;

  public static synchronized Indexer getInstance () {
    if (instance == null) instance = new Indexer();
    return instance;
  }
 private WPI_TalonFX indexer  = new WPI_TalonFX(Constants.indexer.INDEXER_ID, "Pyxis CANivore");
  /** Creates a new Indexer. */
  private Indexer() {
    indexer.configFactoryDefault();
    indexer.configOpenloopRamp(0.25);
    indexer.clearStickyFaults();

    indexer.setStatusFramePeriod(StatusFrame.Status_1_General, 65000);
    indexer.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 2000);
    indexer.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 65000);
    indexer.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 65000);
    indexer.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 65000);
    indexer.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 65000);
    indexer.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 65000);
    indexer.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 65000);
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

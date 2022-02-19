// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private static Indexer indexer;

  public static synchronized Indexer getInstance () {
    if (indexer == null) indexer= new Indexer();
    return indexer;
  }
 private WPI_TalonFX indexerMotor = new WPI_TalonFX(14);
  /** Creates a new Indexer. */
  public Indexer() {}

  @Override
  public void periodic() {
  
  }
  
  public void set(double power){
    indexerMotor.set(power);
  }


}

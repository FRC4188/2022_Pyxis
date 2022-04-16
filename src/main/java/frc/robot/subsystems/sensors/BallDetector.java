// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensors;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallDetector extends SubsystemBase {
  private NetworkTable ballDetectorTable;
  /** Creates a new BallDetector. */
  public BallDetector(String tableName) {
    ballDetectorTable = NetworkTableInstance.getDefault().getTable(tableName);
  }

  public double[] getCenterXs() {
    double[] table = ballDetectorTable.getEntry("center x's").getDoubleArray(new double[0]);
    if (table.length == 0) table = new double[] {0.0};
    return table;
  }

  public double[] getCenterYs() {
    double[] table = ballDetectorTable.getEntry("center y's").getDoubleArray(new double[0]);
    if (table.length == 0) table = new double[] {0.0};
    return table;
  }

  public double[] getSizes() {
    double[] table = ballDetectorTable.getEntry("sizes").getDoubleArray(new double[0]);
    if (table.length == 0) table = new double[] {0.0};
    return table;
  }

  public String[] getNames() {
    String[] table = ballDetectorTable.getEntry("names").getStringArray(new String[0]);
    if (table.length == 0) table = new String[] {""};
    return table;
  }

  public double[] getConfidences() {
    double[] table = ballDetectorTable.getEntry("confidences").getDoubleArray(new double[0]);
    if (table.length == 0) table = new double[] {0.0};
    return table;
  }

  public double getCenterX(int i) {
    return getCenterXs()[i];
  }

  public double getCenterY(int i) {
    return getCenterYs()[i];
  }

  public double getSize(int i) {
    return getSizes()[i];
  }

  public String getName(int i) {
    return getNames()[i];
  }

  public double getConfidence(int i) {
    return getConfidences()[i];
  }

  public double[][] getClosestsIndexes() {
    double[] biggestRed = new double[] {0.0, 0.0};
    double[] biggestBlue = new double[] {0.0, 0.0};

    double[][] biggests = {biggestRed, biggestBlue};

    for (int i = 0; i < getSizes().length; i++) {
      if (getName(i).equals("red ball") && getSize(i) > biggestRed[1]) {
        biggestRed[0] = i;
        biggestRed[1] = getSize(i);
      }
      if (getName(i).equals("blue ball") && getSize(i) > biggestBlue[1]) {
        biggestBlue[0] = i;
        biggestBlue[1] = getSize(i);
      }
    }
    return biggests;
  }
}
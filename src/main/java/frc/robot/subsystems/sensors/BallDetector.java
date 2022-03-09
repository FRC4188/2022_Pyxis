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
    return ballDetectorTable.getEntry("center x's").getDoubleArray(new double[0]);
  }

  public double[] getCenterYs() {
    return ballDetectorTable.getEntry("center y's").getDoubleArray(new double[0]);
  }

  public double[] getSizes() {
    return ballDetectorTable.getEntry("sizes").getDoubleArray(new double[0]);
  }

  public double[] getIDs() {
    return ballDetectorTable.getEntry("ids").getDoubleArray(new double[0]);
  }

  public double[] getConfidences() {
    return ballDetectorTable.getEntry("confidences").getDoubleArray(new double[0]);
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

  public double getID(int i) {
    return getIDs()[i];
  }

  public double getConfidence(int i) {
    return getConfidences()[i];
  }

  public double[] getClosestInfo(DriverStation.Alliance alliance)  {
    double[] ids = getIDs();
    ArrayList<Integer> redIndexes = new ArrayList<Integer>();
    ArrayList<Integer> blueIndexes = new ArrayList<Integer>();

    if (alliance == DriverStation.Alliance.Red || alliance == DriverStation.Alliance.Blue) {
      for (int i = 0; i < ids.length; i++) {
        if (ids[i] == 0.0 && alliance == DriverStation.Alliance.Red) {
          redIndexes.add(i);
        } else if (ids[i] == 1.0 && alliance == DriverStation.Alliance.Blue) {
          blueIndexes.add(i);
        }
      }
    } 

    ArrayList<Integer> processIndexes = (redIndexes.size() > 0) ? redIndexes : blueIndexes;
    int closestIndex = getClosestIndex(processIndexes);

    double[] closest = {getCenterX(closestIndex), getCenterY(closestIndex), getSize(closestIndex), getID(closestIndex), getConfidence(closestIndex)};

    return closest;
  }

  public double xPixelToAngle(double xPixel) {
    return 0.0890625 * xPixel;
  }

  private int getClosestIndex(ArrayList<Integer> processList) {
    double max = 0.0;
    int maxIndex = 0;

    for (int i = 0; i < processList.size(); i++) {
      if (processList.get(i) > max) {
        max = processList.get(i);
        maxIndex = i;
      }
    }
    return maxIndex;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

/** Add your docs here. */
public class Limelight {
    private NetworkTable table;

    public enum LEDMode {
        DEFAULT(0), OFF(1), BLINK(2), ON(3);

        private final int val;
        LEDMode(int val) {
            this.val = val;
        }

        public int getValue() {
            return val;
        }
    }

    public Limelight(String tableName) {
        table = NetworkTableInstance.getDefault().getTable(tableName);
    }

    public void setLEDMode(LEDMode mode) {
        table.getEntry("ledMode").setNumber(mode.getValue());
    }

    public double getTX() {
        return table.getEntry("tx").getDouble(0.0);
    }

    public double getTY() {
        return table.getEntry("ty").getDouble(0.0);
    }

    public double getTS() {
        return table.getEntry("ts").getDouble(0.0);
    }

    public boolean hasTarget() {
        return (((int) table.getEntry("tv").getDouble(0.0)) == 1.0);
    }

    public double[] get3DEstimate() {
        return table.getEntry("camtran").getDoubleArray(new double[]{0.0});
    }

    public double getAngle() {
        return Math.atan2(getTY(), getTX());
    }

    public double getHypotenuse() {
        return Math.hypot(getTY(), getTX());
    }

    public double getHorizontal() {
        return -Math.sin(getAngle() * (getHypotenuse() + (0.164 * getHypotenuse() + 0.102) + Constants.Turret.LL_MOUNTING_ANGLE));
    }

    public double getVertical() {
        return Math.cos(getAngle() * (getHypotenuse() + (0.164 * getHypotenuse() + 0.102) + Constants.Turret.LL_MOUNTING_ANGLE));
    }
}

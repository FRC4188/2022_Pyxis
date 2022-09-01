// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensors;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class Pigeon extends Pigeon2 { 
    public Pigeon(int id, String canBus) {
        super(id, canBus);
    }

    public Pigeon(int id) {
        this(id, "Pyxis CANivore");
    }

    public void init() {
        configFactoryDefault();
        clearStickyFaults();
        getCompassHeading();
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees((getYaw() + 180.0) % 360.0 - 180.0);
    }

    
    public double getRoll() {
        return getRoll();
    }

    public double getPitch() {
        return getPitch();
    }

    public double getTurnRate() {
        double[] xyz_rates = new double[3];
        getRawGyro(xyz_rates);
        return xyz_rates[2];
    }

}

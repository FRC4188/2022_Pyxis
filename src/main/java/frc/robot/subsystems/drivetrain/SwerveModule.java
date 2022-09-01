// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import csplib.motors.CSP_Falcon;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

/** Add your docs here. */
public class SwerveModule {
    private CSP_Falcon speed;
    private CSP_Falcon angle;
    private CANCoder encoder;

    private double zero;

    public SwerveModule(int speedID, int angleID, int encoderID, String canBus, double zero) {
        speed = new CSP_Falcon(speedID);
        angle = new CSP_Falcon(angleID);
        encoder = new CANCoder(encoderID);
        
        this.zero = zero;

        init();
    }

    public SwerveModule(int speedID, int angleID, int encoderID, double zero) {
        this(speedID, angleID, encoderID, "rio", zero);

        init();
    }

    private void init() {
        speed.init();
        angle.init();
        
        encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        encoder.setPosition(0.0);
        encoder.configSensorDirection(false);
        encoder.configMagnetOffset(zero);
        encoder.clearStickyFaults();

        speed.setPIDF(Constants.Drivetrain.SPEED_kP, Constants.Drivetrain.SPEED_kI, Constants.Drivetrain.SPEED_kD, Constants.Drivetrain.SPEED_kF);
        angle.setPIDF(Constants.Drivetrain.ANGLE_kP, Constants.Drivetrain.ANGLE_kI, Constants.Drivetrain.ANGLE_kD, Constants.Drivetrain.ANGLE_kF);
    }

    public void disable() {
        speed.disable();
        angle.disable();
    }

    public void setState(SwerveModuleState desired) {
        SwerveModuleState state = SwerveModuleState.optimize(desired, new Rotation2d(encoder.getPosition()));

        speed.setVelocity(state.speedMetersPerSecond);
        angle.setPosition(state.angle.getDegrees());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(speed.getVelocity(), new Rotation2d(angle.getPosition()));
    }

    public double[] getTemperatures() {
        return new double[]{speed.getTemperature(), angle.getTemperature()};
    }
}

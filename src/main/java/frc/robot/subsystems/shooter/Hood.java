package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class Hood {

    CANSparkMax motor;
    RelativeEncoder encoder;
    //CANCoder encoder;
    PIDController pid = new PIDController(Constants.shooter.hood.kP, Constants.shooter.hood.kI, Constants.shooter.hood.kD);
    private double position = 0.0;
    
    protected Hood(int motorID) {
        motor = new CANSparkMax(motorID, MotorType.kBrushless);
        encoder = motor.getEncoder();
        motor.setInverted(true);
        motor.setIdleMode(IdleMode.kBrake);
        motor.clearFaults();
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 40000);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10000);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);
        motor.setOpenLoopRampRate(0.0125);
        /*
        encoder = new CANCoder(Constants.shooter.hood.ENCODER_ID);
        encoder.configFactoryDefault();
        encoder.setPosition(0.0);
        encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        encoder.configMagnetOffset(Constants.shooter.hood.ZERO);
        encoder.clearStickyFaults();
        encoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 65000);
        */
    }

    public void periodic() {
        setVolts(pid.calculate(getPosition(), position));
    }

    public void setPosition(double position) {
        this.position = position;
    }

    public void resetPosition() {
        encoder.setPosition(0.0);
    }

    public void setVolts(double volts) {
        setVolts(volts, true);
    }

    public void setVolts(double volts, boolean softLimit) {
        if (softLimit) {
            if (getPosition() <= Constants.shooter.hood.MIN && volts < 0.0) motor.set(0.0);
            else if (getPosition() >= Constants.shooter.hood.MAX && volts > 0.0) motor.set(0.0);
            else motor.set(volts / RobotController.getBatteryVoltage());
        } else {
            motor.set(0.0);
            //motor.set(volts / RobotController.getBatteryVoltage());
        }
    }

    public double getPosition() {
        return (encoder.getPosition() / Constants.shooter.hood.GEARING) * 360.0;
    }

    public double getTemp() {
        return motor.getMotorTemperature();
    }

    public double getVelocity() {
        return (encoder.getVelocity() / Constants.shooter.hood.GEARING) * 360.0;
    }
}

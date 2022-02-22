package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class Hood {

    CANSparkMax motor;
    PIDController pid = new PIDController(Constants.shooter.hood.kP, Constants.shooter.hood.kI, Constants.shooter.hood.kD);
    
    protected Hood(int motor) {
        this.motor = new CANSparkMax(motor, MotorType.kBrushless);
    }

    public void setVolts(double volts) {
        if (getPosition() <= Constants.shooter.hood.MIN && volts < 0.0) motor.set(0.0);
        else if (getPosition() >= Constants.shooter.hood.MAX && volts > 0.0) motor.set(0.0);
        else motor.set(volts / RobotController.getBatteryVoltage());
    }

    public void setPosition(double position) {
        setVolts(pid.calculate(getPosition(), position));
    }

    public double getPosition() {
        return 0.0;
    }

    public double getTemp() {
        return motor.getMotorTemperature();
    }
}

package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class ActiveHook {

    private TalonFX motorA = new TalonFX(Constants.climber.MOTOR_A_ID);
    private TalonFX motorB = new TalonFX(Constants.climber.MOTOR_B_ID);

    private CANCoder encoderA = new CANCoder(Constants.climber.ENCODER_A_ID);
    private CANCoder encoderB = new CANCoder(Constants.climber.ENCODER_B_ID);

    private ProfiledPIDController pidA = new ProfiledPIDController(Constants.climber.kP, Constants.climber.kI, Constants.climber.kD, new Constraints(Constants.climber.MAX_VELOCITY, Constants.climber.MAX_ACCEL));
    private ProfiledPIDController pidB = new ProfiledPIDController(Constants.climber.kP, Constants.climber.kI, Constants.climber.kD, new Constraints(Constants.climber.MAX_VELOCITY, Constants.climber.MAX_ACCEL));

    public ActiveHook() {
        resetPositionA(0.0);
        resetPositionB(0.0);
    }

    public void setA(double power) {
        motorA.set(ControlMode.PercentOutput, power / RobotController.getBatteryVoltage());
    }

    public void setB(double power) {
        motorB.set(ControlMode.PercentOutput, power / RobotController.getBatteryVoltage());
    }
    
    /**
     * Method to set the position of the active climber.
     * @param position Position in meters.
     */
    public void setPosition(double position) {
        setA(pidA.calculate(getPositionA(), Math.min(position, Constants.climber.MAX_HEIGHT)));
        setB(pidB.calculate(getPositionB(), Math.min(position, Constants.climber.MAX_HEIGHT)));
    }

    /**
     * Method to get the current position of the active climber.
     * @return Position in meters.
     */
    public double getPositionA() {
        return encoderA.getPosition() * Constants.climber.METERS_PER_COUNT;
    }

    /**
     * Method to get the current position of the active climber.
     * @return Position in meters.
     */
    public double getPositionB() {
        return encoderB.getPosition() * Constants.climber.METERS_PER_COUNT;
    }

    public void resetPositionA(double position) {
        encoderA.setPosition(position / Constants.climber.METERS_PER_COUNT);
    }

    public void resetPositionB(double position) {
        encoderB.setPosition(position / Constants.climber.METERS_PER_COUNT);
    }

    public double getMotorATemp(){
        return motorA.getTemperature();
    }

    public double getMotorBTemp(){
        return motorB.getTemperature();
    }
}

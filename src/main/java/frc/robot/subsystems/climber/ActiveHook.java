package frc.robot.subsystems.climber;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.utils.motors.CSPMotor;

public class ActiveHook {

    private CSPMotor motorA = Constants.devices.climberA;
    private CSPMotor motorB = Constants.devices.climberB;

    private CANCoder encoderA = Constants.devices.encoderA;
    private CANCoder encoderB = Constants.devices.encoderB;

    private ProfiledPIDController pidA = new ProfiledPIDController(Constants.climber.kP, Constants.climber.kI, Constants.climber.kD, new Constraints(Constants.climber.MAX_VELOCITY, Constants.climber.MAX_ACCEL));
    private ProfiledPIDController pidB = new ProfiledPIDController(Constants.climber.kP, Constants.climber.kI, Constants.climber.kD, new Constraints(Constants.climber.MAX_VELOCITY, Constants.climber.MAX_ACCEL));

    protected ActiveHook() {
        resetPositionA(0.0);
        resetPositionB(0.0);

        encoderA.configFactoryDefault();
        encoderB.configFactoryDefault();

        motorA.reset();
        motorB.reset();
        
        motorB.setInverted(false);
        motorA.setInverted(true);

        encoderA.clearStickyFaults();
        encoderB.clearStickyFaults();

        motorBrakes(false);
    }

    public void setA(double power) {
        motorA.set(power / RobotController.getBatteryVoltage());
    }

    public void setB(double power) {
        motorB.set(power / RobotController.getBatteryVoltage());
    }

    /**
     * Method to set the position of the active climber.
     * @param position Position in meters.
     */
    public void setPosition(double position) {
        setA(pidA.calculate(getPositionA(), position));
        setB(pidB.calculate(getPositionB(), position));
    }

    public void motorBrakes(boolean engaged) {
        motorA.setBrake(engaged);
        motorB.setBrake(engaged);
    }

    /**
     * Method to get the current position of the active climber.
     * @return Position in meters.
     */
    public double getPositionA() {
        return motorA.getPosition() * Constants.climber.METERS_PER_COUNT_A;
    }

    /**
     * Method to get the current position of the active climber.
     * @return Position in meters.
     */
    public double getPositionB() {
        return motorB.getPosition() * Constants.climber.METERS_PER_COUNT_B;
    }

    /**
     * Method to get the current position of the active climber.
     * @return Position in meters per second.
     */
    public double getVelocityA() {
        return encoderA.getVelocity() * Constants.climber.METERS_PER_COUNT_A;
    }

    /**
     * Method to get the current position of the active climber.
     * @return Position in meters per second.
     */
    public double getVelocityB() {
        return encoderB.getVelocity() * Constants.climber.METERS_PER_COUNT_B;
    }

    public void resetPositionA(double position) {
        motorA.setEncoder(position / Constants.climber.METERS_PER_COUNT_A);
    }

    public void resetPositionB(double position) {
        motorB.setEncoder(position / Constants.climber.METERS_PER_COUNT_B);
    }

    public double getMotorATemp(){
        return motorA.getTemperature();
    }

    public double getMotorBTemp(){
        return motorB.getTemperature();
    }
}

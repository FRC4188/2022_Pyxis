package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class ActiveHook {

    private TalonFX motorA = new TalonFX(Constants.climber.MOTOR_A_ID);
    private TalonFX motorB = new TalonFX(Constants.climber.MOTOR_B_ID, "Pyxis CANivore");

    private CANCoder encoderA = new CANCoder(Constants.climber.ENCODER_A_ID);
    private CANCoder encoderB = new CANCoder(Constants.climber.ENCODER_B_ID, "Pyxis CANivore");

    private ProfiledPIDController pidA = new ProfiledPIDController(Constants.climber.kP, Constants.climber.kI, Constants.climber.kD, new Constraints(Constants.climber.MAX_VELOCITY, Constants.climber.MAX_ACCEL));
    private ProfiledPIDController pidB = new ProfiledPIDController(Constants.climber.kP, Constants.climber.kI, Constants.climber.kD, new Constraints(Constants.climber.MAX_VELOCITY, Constants.climber.MAX_ACCEL));

    protected ActiveHook() {
        resetPositionA(0.0);
        resetPositionB(0.0);

        encoderA.configFactoryDefault();
        encoderB.configFactoryDefault();

        motorB.setInverted(true);

        motorA.clearStickyFaults();
        motorB.clearStickyFaults();
        encoderA.clearStickyFaults();
        encoderB.clearStickyFaults();
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

    public void motorBrakes(boolean engaged) {
        motorA.setNeutralMode(engaged ? NeutralMode.Brake : NeutralMode.Coast);
        motorB.setNeutralMode(engaged ? NeutralMode.Brake : NeutralMode.Coast);
    }

    /**
     * Method to get the current position of the active climber.
     * @return Position in meters.
     */
    public double getPositionA() {
        return encoderA.getPosition() * Constants.climber.METERS_PER_COUNT_A;
    }

    /**
     * Method to get the current position of the active climber.
     * @return Position in meters.
     */
    public double getPositionB() {
        return encoderB.getPosition() * -Constants.climber.METERS_PER_COUNT_B;
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
        return encoderB.getVelocity() * -Constants.climber.METERS_PER_COUNT_B;
    }

    public void resetPositionA(double position) {
        encoderA.setPosition(position / Constants.climber.METERS_PER_COUNT_A);
    }

    public void resetPositionB(double position) {
        encoderB.setPosition(position / Constants.climber.METERS_PER_COUNT_B);
    }

    public double getMotorATemp(){
        return motorA.getTemperature();
    }

    public double getMotorBTemp(){
        return motorB.getTemperature();
    }

    public double getMotorACurrent(){
        return motorA.getStatorCurrent();
    }

    public double getMotorBCurrent(){
        return motorB.getStatorCurrent();
    }
}

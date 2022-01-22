package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.Constants;
import frc.robot.ModuleConstants;
import frc.robot.utils.controllers.ContinuousTrapezoid;
import frc.robot.utils.controllers.SSMPosition;
import frc.robot.utils.controllers.SSMVelocity;

/**
 * Class to define Swerve Module Control code.
 */
public class Module {

    private WPI_TalonFX speedMotor = null;
    private SSMVelocity speedController = null;
    private ContinuousTrapezoid speedProfile = null;
    private WPI_TalonFX angleMotor = null;
    private SSMPosition angleController = null;
    private ContinuousTrapezoid angleProfile = null;
    private CANCoder encoder = null;

    /**
     * Constructs a new Module object.
     * Controls the velocity and angle of swerve wheels.
     * @param speedID CAN ID of the speed motor.
     * @param angleID CAN ID of the angle motor.
     * @param encoderID CAN ID of the CANCoder.
     * @param encoderZero Magnet offset angle of the CANCoder.
     */
    public Module(int module) {
        //ModuleConstants constants = ModuleConstants.modules[module];

        speedMotor = new WPI_TalonFX(module*2 + 1);
        angleMotor = new WPI_TalonFX(module*2);
        encoder = new CANCoder(20+module);

        speedMotor.configFactoryDefault();
        speedMotor.setNeutralMode(NeutralMode.Brake);
        speedMotor.configClosedloopRamp(0.5);
        speedMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        speedMotor.setSelectedSensorPosition(0);
        speedMotor.setInverted(true);

        angleMotor.configFactoryDefault();
        angleMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        angleMotor.setSelectedSensorPosition(0);

        encoder.configFactoryDefault();
        encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        encoder.setPosition(0.0);
        encoder.configSensorDirection(false);
        encoder.configMagnetOffset(0.0);

        speedController = new SSMVelocity(0.01, 0.01, 5.0, 0.15, 5e3, 1e-5);
        speedProfile = new ContinuousTrapezoid(new Constraints(4.0, 12.0));
        angleController = new SSMPosition(1.1059E-49, 5.1324E-51,  VecBuilder.fill(270.0, 360.0), VecBuilder.fill(0.125), VecBuilder.fill(3.6e4, 3.6e3), VecBuilder.fill(3.6e-5));
        angleProfile = new ContinuousTrapezoid(new Constraints(720.0, 1440.0));
    }

    /**
     * Returns the measured angle of the angle motor.
     * @return Measured angle.
     */
    public double getAbsoluteAngle() {
        return encoder.getAbsolutePosition();
    }

    /**
     * Returns the measured meters per second velocity of the speed motor.
     * @return Measured meters per second.
     */
    public double getVelocity() {
        return ((double) speedMotor.getSelectedSensorVelocity() * 10.0) / Constants.drive.DRIVE_COUNTS_PER_METER;
    }

    double lastAngle = 0.0;
    /**
     * Set the angle and velocity of the swerve module.
     * @param state State object containing desired module bahaviour.
     */
    public void setModuleState(SwerveModuleState state) {
        double setVelocity = state.speedMetersPerSecond;
        double setAngle = state.angle.getDegrees();
        
        if (Math.abs(lastAngle - setAngle) > 90) {
            setAngle = (setAngle + 360) % 360 - 180;
            setVelocity *= -1.0;
        }

        setVelocity = speedProfile.calculate(getVelocity(), setVelocity).position;
        State pSetAngle = angleProfile.calculate(getAbsoluteAngle(), setAngle);
        
        speedMotor.set(ControlMode.PercentOutput, speedController.calculate(getVelocity(), setVelocity) / RobotController.getBatteryVoltage());
        angleMotor.set(ControlMode.PercentOutput, angleController.calculate(getAbsoluteAngle(), VecBuilder.fill(pSetAngle.position, pSetAngle.velocity)) / RobotController.getBatteryVoltage());

        lastAngle = getAbsoluteAngle();
    }

    /**
     * Get the angle and velocity of the swerve module.
     * @return Current module state.
     */
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getVelocity(), Rotation2d.fromDegrees(getAbsoluteAngle()));
    }

    /**
     * The temperature of the speed motor.
     * @return Temperature in celsius.
     */
    public double getSpeedTemp() {
    return speedMotor.getTemperature();
    }

    /**
     * The temperature of the angle motor.
     * @return Temperature in celsius.
     */
    public double getAngleTemp() {
    return angleMotor.getTemperature();
    }

    public double getSpeedVoltage() {
        return speedMotor.get() * RobotController.getInputVoltage();
    }

    public double getAngleVoltage() {
        return angleMotor.get() * RobotController.getInputVoltage();
    }
}

package frc.robot.utils.controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.RobotController;

public class PIDFPosController {

    private PIDController pid = null;
    private SimpleMotorFeedforward ff = null;
    
    public PIDFPosController(double kP, double kI, double kD, double kS, double kV) {
        pid = new PIDController(kP, kI, kD);
        ff = new SimpleMotorFeedforward(kS, kV);
    }

    public double calculate(double measurement, State set) {
        return pid.calculate(measurement, set.position) + ff.calculate(set.velocity) / RobotController.getInputVoltage();
    }
}

package frc.robot.utils.controllers;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;

public class SSMVelocity {
    private final LinearSystem<N1, N1, N1> plant;
    private final KalmanFilter<N1, N1, N1> observer;
    private final LinearQuadraticRegulator<N1, N1, N1> controller;
    private final LinearSystemLoop<N1, N1, N1> loop;

    /**
     * Constructs a State-Space based controller for a velocity system.
     * @param kV kV Feed Forward constant of the system.
     * @param kA kA Feed Forward constant of the system.
     * @param Q Qelm term for the controller; measurement error tolerance.
     * @param R Relm term for the controller; control effort tolerance.
     * @param stateStDev State standard deviation in the model predictions.
     * @param measureStDev Measurement standard deviation.
     */
    public SSMVelocity(double kV, double kA, double Q, double R, double stateStDev, double measureStDev) {
        this.plant = LinearSystemId.identifyVelocitySystem(kV, kA);
        this.observer = new KalmanFilter<N1, N1, N1>(Nat.N1(), Nat.N1(), plant, VecBuilder.fill(stateStDev), VecBuilder.fill(measureStDev), 0.02);
        this.controller = new LinearQuadraticRegulator<>(plant, VecBuilder.fill(Q), VecBuilder.fill(R), 0.02);
        this.loop = new LinearSystemLoop<>(plant, controller, observer, 12.0, 0.02);
    }

    /**
     * Update the model and predict optimal next voltage.
     * @param measurement Measured velocity of the system.
     * @param set Desired velocity of the system.
     * @return Calculated next voltage of the system.
     */
    public double calculate(double measurement, double set) {
        //loop.reset(VecBuilder.fill(measurement));
        loop.setNextR(VecBuilder.fill(set));
        loop.correct(VecBuilder.fill(measurement));
        loop.predict(0.02);
        return loop.getU(0);
    }
}

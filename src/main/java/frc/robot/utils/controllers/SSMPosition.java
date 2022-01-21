package frc.robot.utils.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;

public class SSMPosition {
    private final LinearSystem<N2, N1, N1> plant;
    private final KalmanFilter<N2, N1, N1> observer;
    private final LinearQuadraticRegulator<N2, N1, N1> controller;
    private final LinearSystemLoop<N2, N1, N1> loop;

    private boolean continuous = false;
    private double min = -1.0;
    private double max = 1.0;

    /**
     * Constructs a State-Space based controller for position.
     * @param kV kV Feed Forward constant of the system.
     * @param kA kA Feed Forward constant of the system.
     * @param Q Qelm vector of the system; measurement error tolerance.
     * @param R Relm vector of the system; control effort tolerance.
     * @param stateStDev State standard deviation vector of the system.
     * @param measureStDev Measurement standard deviation of the system.
     */
    public SSMPosition(double kV, double kA, Vector<N2> Q, Vector<N1> R, Vector<N2> stateStDev, Vector<N1> measureStDev) {
        this.plant = LinearSystemId.identifyPositionSystem(kV, kA);
        this.observer = new KalmanFilter<N2, N1, N1>(Nat.N2(), Nat.N1(), plant, stateStDev, measureStDev, 0.02);
        this.controller = new LinearQuadraticRegulator<N2, N1, N1>(plant, Q, R, 0.02);
        this.loop = new LinearSystemLoop<>(plant, controller, observer, 12.0, 0.02);
    }

    /**
     * Update the model and get the next set voltage.
     * @param measurement Position measurement of the system.
     * @param set Set vector of the system (position, velocity).
     * @return Next input voltage of the system.
     */
    public double calculate(double measurement, Vector<N2> set) {
        if (continuous) {
            double errorBound = (max - min) / 2.0;
            set.set(0, 0, measurement + MathUtil.inputModulus(set.get(0, 0) - measurement, -errorBound, errorBound));
        }

        loop.setNextR(set);
        loop.correct(VecBuilder.fill(measurement));
        loop.predict(0.02);
        return loop.getU(0);
    }

    public void setContinuous(boolean continuous) {
        this.continuous = continuous;
    }

    public void setContinuous(boolean continuous, double min, double max) {
        this.continuous = continuous;
        this.min = min;
        this.max = max;
    }
}

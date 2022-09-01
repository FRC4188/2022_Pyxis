// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import csplib.motors.CSP_Falcon;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.robot.Constants;

/** Add your docs here. */
public class Flywheel {
    private CSP_Falcon leader;
    private CSP_Falcon follower;

    private LinearSystem<N1, N1, N1> plant = LinearSystemId.identifyVelocitySystem(Constants.Shooter.kV, Constants.Shooter.kA);
    private KalmanFilter<N1, N1, N1> filter = new KalmanFilter<>(Nat.N1(), Nat.N1(), plant, VecBuilder.fill(Constants.Shooter.ACCURACY), VecBuilder.fill(0.001), 0.02);
    private LinearQuadraticRegulator<N1, N1, N1> controller = new LinearQuadraticRegulator<>(plant, VecBuilder.fill(Constants.Shooter.QELMS), VecBuilder.fill(12.0), 0.02);
    private LinearSystemLoop<N1, N1, N1> loop = new LinearSystemLoop<>(plant, controller, filter, 12.0, 0.02);

    public Flywheel(int leaderID, int followerID) {
        leader = new CSP_Falcon(leaderID);
        follower = new CSP_Falcon(followerID);

        init(); 
        
    }

    private void init() {
        leader.init();
        follower.init();

        follower.follow(leader);        

        loop.reset(VecBuilder.fill(leader.getVelocity()));
    }

    public void set(double percentage) {
        leader.set(percentage);
    }

    public void setVoltage(double voltage) {
        leader.setVoltage(voltage);
    }

    public void setVelocity(double velocity) {
        loop.setNextR(VecBuilder.fill(velocity));
        loop.correct(VecBuilder.fill(leader.getVelocity()));
        loop.predict(0.02);

        setVoltage(loop.getU(0));
    }

    public double getVelocity() {
        return leader.getVelocity();
    }

    public double getInputVoltage() {
        return loop.getU(0);
    }

    public double getEstimatedVelocity() {
        return loop.getXHat(0);
    }   

    public double getNextReference() {
        return loop.getNextR(0);
    }

    public double[] getTemperatures() {
        return new double[]{leader.getTemperature(), follower.getTemperature()};
    }
}

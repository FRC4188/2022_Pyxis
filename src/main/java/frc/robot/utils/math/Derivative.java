package frc.robot.utils.math;

public class Derivative {
    private static double lastTime = System.currentTimeMillis();
    private static double lastVal;

    public Derivative(double start) {
        lastVal = start;
    }

    public double getRate(double value) {
        double time = System.currentTimeMillis();
        double rate = (value - lastVal) / ((time - lastTime) / 1000.0);

        lastTime = time;
        lastVal = value;

        return rate;
    }
}

package frc.robot.math;

import java.util.ArrayList;

public class Derivative {
  private static double lastTime = System.currentTimeMillis();
  private static double lastVal;

  public Derivative(double start) {
    lastVal = start;
  }

  public static double getRate(double ds) {
    double time = System.currentTimeMillis();
    double rate = (ds - lastVal) / ((time - lastTime) / 1000.0);

    lastTime = time;
    lastVal = ds;

    return rate;
  }

  public static double getRate(double ds, int order) {
    if (order > 0) {
      return getRate(order - 1);
    } else {
      return 0.0;
    }
  }

  public static double[] getRates(double[] values) {
    ArrayList<Double> ratesList = new ArrayList<Double>();
    for (double value : values) {
      ratesList.add(getRate(value));
    }
    return ratesList.stream().mapToDouble(d -> d).toArray();
  }
}

package frc.robot.math;

public class LogisticController {

  private double P;
  private double D;
  private double S;

  /**
   * Constructs a new LogisticController to close error in a non-linear system.
   *
   * @param proportion The value in range [0.0, 1.0] to determine the base response before
   *     deviation.
   * @param deviation The value in range [0.0, 1.0] to determine the "stretch" of the logistic
   *     adjustment.
   * @param squaring The value in range [0.0, 1.0] to determine the rate of dropoff in the logistic
   *     adjustment.
   */
  public LogisticController(double proportion, double deviation, double squaring) {
    P = proportion;
    D = deviation;
    S = squaring;
  }

  public double calculate(double set, double measure) {
    double error = set - measure;

    double logis1 = -2.0 / (1 + Math.pow(2.0, -error * ((1.0 - D) / D)));
    double logis2 = 2.0 / (1 + Math.pow(2.0, error * ((1.0 - D) / D)));

    double adjustment = Math.pow(logis1 + logis2 + 1.0, S / (1.0 - S));

    double result = (P / (1.0 - P)) * error * adjustment;

    result = result < -1.0 ? -1.0 : result;
    result = result > 1.0 ? 1.0 : result;

    return result;
  }
}

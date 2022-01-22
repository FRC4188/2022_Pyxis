package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public final class ModuleConstants {
    public static double ZERO[] = {0.0, 0.0, 0.0, 0.0};
    public static int SPEED_ID[] = {2, 4, 5, 8};
    public static int ANGLE_ID[] = {1, 2, 3, 4};
    public static int ENC_ID[] = {21, 22, 23, 24};

    public static double ANGLE_MAX_VEL = 720.0;
    public static double ANGLE_MAX_ACCEL = 1440.0;
    public static double SPEED_MAX_ACCEL = 4.0;
    public static double SPEED_MAX_JERK = 12.0;

    /** kV Feed Forward constant for the angle */
    public static double ANGLE_kV = 1.1059E-49;
    /** kA Feed Forward constant for the angle */
    public static double ANGLE_kA = 5.1324E-51;

    /** Standard deviation in state predictions for the module angle. */
    public static Vector<N2> ANGLE_STATE_STDEV = VecBuilder.fill(3.6e4, 3.6e3);
    /** Standard deviaiton in the encoder measurements for the module angle. */
    public static Vector<N1> ANGLE_ENC_STDEV = VecBuilder.fill(3.6e-5);

    /** Qelm constant; measurement error tolerance. */
    public static Vector<N2> ANGLE_QELMS = VecBuilder.fill(270.0, 360.0);
    /** Relm constant; control effort tolerance. */
    public static Vector<N1> ANGLE_RELMS = VecBuilder.fill(0.125);

    /** kV Feed Forward constant for the speed */
    public static double SPEED_kV = 0.1;
    /** kA Feed Forward constant for the speed */
    public static double SPEED_kA = 0.1;

    /** Standard deviation in state predictions for the module speed. */
    public static double SPEED_STATE_STDEV = 5e4;
    /** Standard deviaiton in the encoder measurements for the module speed. */
    public static double SPEED_ENC_STDEV = 1e-5;

    /** Qelm constant; measurement error tolerance. */
    public static double SPEED_QELMS = 2.0;
    /** Relm constant; control effort tolerance. */
    public static double SPEED_RELMS = 0.15;
}

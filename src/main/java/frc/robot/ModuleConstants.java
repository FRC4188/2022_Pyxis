package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class ModuleConstants {
    public double ZERO = 0;
    public int SPEED_ID = 0;
    public int ANGLE_ID = 0;
    public int ENC_ID = 0;

    public double ANGLE_MAX_VEL = 720.0;
    public double ANGLE_MAX_ACCEL = 1440.0;
    public double SPEED_MAX_ACCEL = 4.0;
    public double SPEED_MAX_JERK = 12.0;

    /** kV Feed Forward constant for the angle */
    public double ANGLE_kV = 0.1;
    /** kA Feed Forward constant for the angle */
    public double ANGLE_kA = 0.1;

    /** Standard deviation in state predictions for the module angle. */
    public Vector<N2> ANGLE_STATE_STDEV = VecBuilder.fill(0, 0);
    /** Standard deviaiton in the encoder measurements for the module angle. */
    public Vector<N1> ANGLE_ENC_STDEV = VecBuilder.fill(0);

    /** Qelm constant; measurement error tolerance. */
    public Vector<N2> ANGLE_QELMS = VecBuilder.fill(0, 0);
    /** Relm constant; control effort tolerance. */
    public Vector<N1> ANGLE_RELMS = VecBuilder.fill(0);

    /** kV Feed Forward constant for the speed */
    public double SPEED_kV = 0.1;
    /** kA Feed Forward constant for the speed */
    public double SPEED_kA = 0.1;

    /** Standard deviation in state predictions for the module speed. */
    public double SPEED_STATE_STDEV = 0;
    /** Standard deviaiton in the encoder measurements for the module speed. */
    public double SPEED_ENC_STDEV = 0;

    /** Qelm constant; measurement error tolerance. */
    public double SPEED_QELMS = 0;
    /** Relm constant; control effort tolerance. */
    public double SPEED_RELMS = 0;

    public static final ModuleConstants[] modules = new ModuleConstants[4];

    static {
        for (int i = 0; i < modules.length; i++) modules[i] = new ModuleConstants();

        modules[0].ANGLE_ID = 1;
        modules[0].SPEED_ID = 2;
        modules[0].ENC_ID = 21;
        modules[0].ZERO = 0.0;

        modules[1].ANGLE_ID = 3;
        modules[1].SPEED_ID = 4;
        modules[1].ENC_ID = 22;
        modules[1].ZERO = 0.0;

        modules[2].ANGLE_ID = 5;
        modules[2].SPEED_ID = 6;
        modules[2].ENC_ID = 23;
        modules[2].ZERO = 0.0;

        modules[3].ANGLE_ID = 7;
        modules[3].SPEED_ID = 8;
        modules[3].ENC_ID = 24;
        modules[3].ZERO = 0.0;
    }
}

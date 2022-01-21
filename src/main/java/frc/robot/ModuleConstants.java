package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class ModuleConstants {
    public final double ZERO = 0;
    public final int SPEED_ID = 0;
    public final int ANGLE_ID = 0;
    public final int ENC_ID = 0;

    public final double ANGLE_MAX_VEL = 720.0;
    public final double ANGLE_MAX_ACCEL = 1440.0;
    public final double SPEED_MAX_ACCEL = 4.0;
    public final double SPEED_MAX_JERK = 12.0;

    // /** kV Feed Forward constant for the angle */
    // public final double ANGLE_kV = 0;
    // /** kA Feed Forward constant for the angle */
    // public final double ANGLE_kA = 0;

    // /** Standard deviation in state predictions for the module angle. */
    // public final Vector<N2> ANGLE_STATE_STDEV = VecBuilder.fill(0, 0);
    // /** Standard deviaiton in the encoder measurements for the module angle. */
    // public final Vector<N1> ANGLE_ENC_STDEV = VecBuilder.fill(0);

    // /** Qelm constant; measurement error tolerance. */
    // public final Vector<N2> ANGLE_QELMS = VecBuilder.fill(0, 0);
    // /** Relm constant; control effort tolerance. */
    // public final Vector<N1> ANGLE_RELMS = VecBuilder.fill(0);

    // /** kV Feed Forward constant for the speed */
    // public final double SPEED_kV = 0;
    // /** kA Feed Forward constant for the speed */
    // public final double SPEED_kA = 0;

    // /** Standard deviation in state predictions for the module speed. */
    // public final double SPEED_STATE_STDEV = 0;
    // /** Standard deviaiton in the encoder measurements for the module speed. */
    // public final double SPEED_ENC_STDEV = 0;

    // /** Qelm constant; measurement error tolerance. */
    // public final double SPEED_QELMS = 0;
    // /** Relm constant; control effort tolerance. */
    // public final double SPEED_RELMS = 0;
}
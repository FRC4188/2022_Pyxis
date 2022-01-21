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
import frc.robot.ModuleConstants;

final class module1 extends ModuleConstants {
    /** Raw encoder reading when the module is at 0 degrees. */
    private static final double ZERO = 0.0;
    private static final int SPEED_ID = 1;
    private static final int ANGLE_ID = 2;
    private static final int ENC_ID = 11;

    /** kV Feed Forward constant for the angle */
    private final double ANGLE_kV = 1.1059E-49;
    /** kA Feed Forward constant for the angle */
    private final double ANGLE_kA = 1.1059E-49;

    /** Standard deviation in state predictions for the module angle. */
    private final Vector<N2> ANGLE_STATE_STDEV = VecBuilder.fill(3.6e7, 3.6e6);
    /** Standard deviaiton in the encoder measurements for the module angle. */
    private final Vector<N1> ANGLE_ENC_STDEV = VecBuilder.fill(1e-4);

    /** Qelm constant; measurement error tolerance. */
    private final Vector<N2> ANGLE_QELMS = VecBuilder.fill(65.0, 135.0);
    /** Relm constant; control effort tolerance. */
    private final Vector<N1> ANGLE_RELMS = VecBuilder.fill(0.125);

    /** kV Feed Forward constant for the speed */
    private final double SPEED_kV = 0.1;
    /** kA Feed Forward constant for the speed */
    private final double SPEED_kA = 0.1;

    /** Standard deviation in state predictions for the module speed. */
    private final double SPEED_STATE_STDEV = 5e4;
    /** Standard deviaiton in the encoder measurements for the module speed. */
    private final double SPEED_ENC_STDEV = 1e-4;

    /** Qelm constant; measurement error tolerance. */
    private final double SPEED_QELMS = 1.5;
    /** Relm constant; control effort tolerance. */
    private final double SPEED_RELMS = 0.25;
}
final class module2 extends ModuleConstants {
    /** Raw encoder reading when the module is at 0 degrees. */
    private static final double ZERO = 0.0;
    private static final int SPEED_ID = 1;
    private static final int ANGLE_ID = 2;
    private static final int ENC_ID = 11;

    /** kV Feed Forward constant for the angle */
    private final double ANGLE_kV = 0.1;
    /** 1.1059E-49Feed Forward constant for the angle */
    private final double ANGLE_kA = 5.1324E-51;

    /** Standard deviation in state predictions for the module angle. */
    private final Vector<N2> ANGLE_STATE_STDEV = VecBuilder.fill(3.6e7, 3.6e6);
    /** Standard deviaiton in the encoder measurements for the module angle. */
    private final Vector<N1> ANGLE_ENC_STDEV = VecBuilder.fill(1e-4);

    /** Qelm constant; measurement error tolerance. */
    private final Vector<N2> ANGLE_QELMS = VecBuilder.fill(65.0, 135.0);
    /** Relm constant; control effort tolerance. */
    private final Vector<N1> ANGLE_RELMS = VecBuilder.fill(0.125);

    /** kV Feed Forward constant for the speed */
    private final double SPEED_kV = 0.1;
    /** kA Feed Forward constant for the speed */
    private final double SPEED_kA = 0.1;

    /** Standard deviation in state predictions for the module speed. */
    private final double SPEED_STATE_STDEV = 5e4;
    /** Standard deviaiton in the encoder measurements for the module speed. */
    private final double SPEED_ENC_STDEV = 1e-4;

    /** Qelm constant; measurement error tolerance. */
    private final double SPEED_QELMS = 1.5;
    /** Relm constant; control effort tolerance. */
    private final double SPEED_RELMS = 0.25;}
final class module3 extends ModuleConstants {
    /** Raw encoder reading when the module is at 0 degrees. */
    private static final double ZERO = 0.0;
    private static final int SPEED_ID = 1;
    private static final int ANGLE_ID = 2;
    private static final int ENC_ID = 11;

    /** kV Feed Forward constant for the angle */
    private final double ANGLE_kV = 1.1059E-49;
    /** kA Feed Forward constant for the angle */
    private final double ANGLE_kA = 5.1324E-51;

    /** Standard deviation in state predictions for the module angle. */
    private final Vector<N2> ANGLE_STATE_STDEV = VecBuilder.fill(3.6e7, 3.6e6);
    /** Standard deviaiton in the encoder measurements for the module angle. */
    private final Vector<N1> ANGLE_ENC_STDEV = VecBuilder.fill(1e-4);

    /** Qelm constant; measurement error tolerance. */
    private final Vector<N2> ANGLE_QELMS = VecBuilder.fill(65.0, 135.0);
    /** Relm constant; control effort tolerance. */
    private final Vector<N1> ANGLE_RELMS = VecBuilder.fill(0.125);

    /** kV Feed Forward constant for the speed */
    private final double SPEED_kV = 0.1;
    /** kA Feed Forward constant for the speed */
    private final double SPEED_kA = 0.1;

    /** Standard deviation in state predictions for the module speed. */
    private final double SPEED_STATE_STDEV = 5e4;
    /** Standard deviaiton in the encoder measurements for the module speed. */
    private final double SPEED_ENC_STDEV = 1e-4;

    /** Qelm constant; measurement error tolerance. */
    private final double SPEED_QELMS = 1.5;
    /** Relm constant; control effort tolerance. */
    private final double SPEED_RELMS = 0.25;}
final class module4 extends ModuleConstants {
    /** Raw encoder reading when the module is at 0 degrees. */
    private static final double ZERO = 0.0;
    private static final int SPEED_ID = 1;
    private static final int ANGLE_ID = 2;
    private static final int ENC_ID = 11;

    /** kV Feed Forward constant for the angle */
    private final double ANGLE_kV = 1.1059E-49;
    /** kA Feed Forward constant for the angle */
    private final double ANGLE_kA = 5.1324E-51;

    /** Standard deviation in state predictions for the module angle. */
    private final Vector<N2> ANGLE_STATE_STDEV = VecBuilder.fill(3.6e7, 3.6e6);
    /** Standard deviaiton in the encoder measurements for the module angle. */
    private final Vector<N1> ANGLE_ENC_STDEV = VecBuilder.fill(1e-4);

    /** Qelm constant; measurement error tolerance. */
    private final Vector<N2> ANGLE_QELMS = VecBuilder.fill(65.0, 135.0);
    /** Relm constant; control effort tolerance. */
    private final Vector<N1> ANGLE_RELMS = VecBuilder.fill(0.125);

    /** kV Feed Forward constant for the speed */
    private final double SPEED_kV = 0.1;
    /** kA Feed Forward constant for the speed */
    private final double SPEED_kA = 0.1;

    /** Standard deviation in state predictions for the module speed. */
    private final double SPEED_STATE_STDEV = 5e4;
    /** Standard deviaiton in the encoder measurements for the module speed. */
    private final double SPEED_ENC_STDEV = 1e-4;

    /** Qelm constant; measurement error tolerance. */
    private final double SPEED_QELMS = 1.5;
    /** Relm constant; control effort tolerance. */
    private final double SPEED_RELMS = 0.25;
}

public class Modules {
    public static final ModuleConstants[] modules = {new module1(), new module2(), new module3(), new module4()};
}
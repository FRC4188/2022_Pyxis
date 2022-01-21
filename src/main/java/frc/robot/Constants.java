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

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {

    public final class robot {
            public static final double A_LENGTH = 0.59055; // Axel length (Meters).
            public static final double A_WIDTH = 0.48895; // Axel width (Meters).
            public final double A_CROSSLENGTH = Math.hypot(A_LENGTH, A_WIDTH);

            public static final double FALCON_ENCODER_TICKS = 2048.0; //Counts per revolution of the Falcon 500 motor.
            public static final double FALCON_MAX_TEMP = 50.0; //Max temperature of Falcon 500 (Celsius).
            public static final double FALCON_MAX_VEL = 6380.0;
        }

        public class drive {
            public final double DRIVE_GEARING = 6.92; // Gear ratio of the drive motor.
            public final double WHEEL_DIAMETER = Units.inchesToMeters(4); //Diameter of the drive wheels (Meters).
            public final double WHEEL_CIRCUMFRENCE = Math.PI * WHEEL_DIAMETER; // Circumfrence of the drive wheels (Meters).
            public final double DRIVE_ROTATIONS_PER_METER = 1.0 / WHEEL_CIRCUMFRENCE; // Rotations per meter of the drive wheels.
            public final double DRIVE_COUNTS_PER_ROTATION = DRIVE_GEARING * robot.FALCON_ENCODER_TICKS; // Encoder counts per revolution of the drive wheel.
            public final double DRIVE_COUNTS_PER_METER = DRIVE_ROTATIONS_PER_METER * DRIVE_COUNTS_PER_ROTATION; // Encoder ticks per meter of the drive wheels.

            public final double ANGLE_GEARING = 11.57;
            public final double ANGLE_TICKS_PER_DEGREE = (ANGLE_GEARING * robot.FALCON_ENCODER_TICKS) / 360.0;

            public final double MAX_VOLTS = 12.0; // Maximum voltage allowed in the drivetrain.
            public final double MAX_VELOCITY = 10.0; // Maximum velocity allowed in the drivetrain (Meters per Second).
            public final double MAX_ACCEL = 20.0; // Maximum acceleration of the drivetrain in (Meters per Second Squared).
            public final double MAX_CACCEL = 8.0; // Maximum centripital acceleration of the robot (Meters per Second Squared).
            public final double MAX_RADIANS = 3.0 * Math.PI; // Maximum rotational velocity (Radians per Second).

            public final double ROTATION_KV = 0.0;
            public final double ROTATION_KA = 0.0;

            // Put together swerve module positions relative to the center of the robot.
            public final Translation2d FrontLeftLocation = new Translation2d(-(Constants.robot.A_WIDTH / 2), -(Constants.robot.A_LENGTH / 2));
            public final Translation2d FrontRightLocation = new Translation2d(-(Constants.robot.A_WIDTH / 2), (Constants.robot.A_LENGTH / 2));
            public final Translation2d BackLeftLocation = new Translation2d((Constants.robot.A_WIDTH / 2), -(Constants.robot.A_LENGTH / 2));
            public final Translation2d BackRightLocation = new Translation2d((Constants.robot.A_WIDTH / 2), (Constants.robot.A_LENGTH / 2));

            public class ModuleConstants {
                public final double ZERO = 0;
                public final int SPEED_ID = 0;
                public final int ANGLE_ID = 0;
                public final int ENC_ID = 0;

                public final double ANGLE_MAX_VEL = 720.0;
                public final double ANGLE_MAX_ACCEL = 1440.0;
                public final double SPEED_MAX_ACCEL = 4.0;
                public final double SPEED_MAX_JERK = 12.0;

                /** kV Feed Forward constant for the angle */
                public final double ANGLE_kV = 0;
                /** kA Feed Forward constant for the angle */
                public final double ANGLE_kA = 0;

                /** Standard deviation in state predictions for the module angle. */
                public final Vector<N2> ANGLE_STATE_STDEV = VecBuilder.fill(0, 0);
                /** Standard deviaiton in the encoder measurements for the module angle. */
                public final Vector<N1> ANGLE_ENC_STDEV = VecBuilder.fill(0);

                /** Qelm constant; measurement error tolerance. */
                public final Vector<N2> ANGLE_QELMS = VecBuilder.fill(0, 0);
                /** Relm constant; control effort tolerance. */
                public final Vector<N1> ANGLE_RELMS = VecBuilder.fill(0);

                /** kV Feed Forward constant for the speed */
                public final double SPEED_kV = 0;
                /** kA Feed Forward constant for the speed */
                public final double SPEED_kA = 0;

                /** Standard deviation in state predictions for the module speed. */
                public final double SPEED_STATE_STDEV = 0;
                /** Standard deviaiton in the encoder measurements for the module speed. */
                public final double SPEED_ENC_STDEV = 0;

                /** Qelm constant; measurement error tolerance. */
                public final double SPEED_QELMS = 0;
                /** Relm constant; control effort tolerance. */
                public final double SPEED_RELMS = 0;
            }

            public final class module1 extends ModuleConstants {
                /** Raw encoder reading when the module is at 0 degrees. */
                public static final double ZERO = 0.0;
                public static final int SPEED_ID = 1;
                public static final int ANGLE_ID = 2;
                public static final int ENC_ID = 11;

                /** kV Feed Forward constant for the angle */
                public final double ANGLE_kV = 0;
                /** kA Feed Forward constant for the angle */
                public final double ANGLE_kA = 0;

                /** Standard deviation in state predictions for the module angle. */
                public final Vector<N2> ANGLE_STATE_STDEV = VecBuilder.fill(0, 0);
                /** Standard deviaiton in the encoder measurements for the module angle. */
                public final Vector<N1> ANGLE_ENC_STDEV = VecBuilder.fill(0);

                /** Qelm constant; measurement error tolerance. */
                public final Vector<N2> ANGLE_QELMS = VecBuilder.fill(0, 0);
                /** Relm constant; control effort tolerance. */
                public final Vector<N1> ANGLE_RELMS = VecBuilder.fill(0);

                /** kV Feed Forward constant for the speed */
                public final double SPEED_kV = 0;
                /** kA Feed Forward constant for the speed */
                public final double SPEED_kA = 0;

                /** Standard deviation in state predictions for the module speed. */
                public final double SPEED_STATE_STDEV = 0;
                /** Standard deviaiton in the encoder measurements for the module speed. */
                public final double SPEED_ENC_STDEV = 0;

                /** Qelm constant; measurement error tolerance. */
                public final double SPEED_QELMS = 0;
                /** Relm constant; control effort tolerance. */
                public final double SPEED_RELMS = 0;
            }
            public final class module2 extends ModuleConstants {
                /** Raw encoder reading when the module is at 0 degrees. */
                public static final double ZERO = 0.0;
                public static final int SPEED_ID = 3;
                public static final int ANGLE_ID = 4;
                public static final int ENC_ID = 12;

                /** kV Feed Forward constant for the angle */
                public final double ANGLE_kV = 0;
                /** kA Feed Forward constant for the angle */
                public final double ANGLE_kA = 0;

                /** Standard deviation in state predictions for the module angle. */
                public final Vector<N2> ANGLE_STATE_STDEV = VecBuilder.fill(0, 0);
                /** Standard deviaiton in the encoder measurements for the module angle. */
                public final Vector<N1> ANGLE_ENC_STDEV = VecBuilder.fill(0);

                /** Qelm constant; measurement error tolerance. */
                public final Vector<N2> ANGLE_QELMS = VecBuilder.fill(0, 0);
                /** Relm constant; control effort tolerance. */
                public final Vector<N1> ANGLE_RELMS = VecBuilder.fill(0);

                /** kV Feed Forward constant for the speed */
                public final double SPEED_kV = 0;
                /** kA Feed Forward constant for the speed */
                public final double SPEED_kA = 0;

                /** Standard deviation in state predictions for the module speed. */
                public final double SPEED_STATE_STDEV = 0;
                /** Standard deviaiton in the encoder measurements for the module speed. */
                public final double SPEED_ENC_STDEV = 0;

                /** Qelm constant; measurement error tolerance. */
                public final double SPEED_QELMS = 0;
                /** Relm constant; control effort tolerance. */
                public final double SPEED_RELMS = 0;
            }
            public final class module3 extends ModuleConstants {
                /** Raw encoder reading when the module is at 0 degrees. */
                public static final double ZERO = 0.0;
                public static final int SPEED_ID = 5;
                public static final int ANGLE_ID = 6;
                public static final int ENC_ID = 13;

                /** kV Feed Forward constant for the angle */
                public final double ANGLE_kV = 0;
                /** kA Feed Forward constant for the angle */
                public final double ANGLE_kA = 0;

                /** Standard deviation in state predictions for the module angle. */
                public final Vector<N2> ANGLE_STATE_STDEV = VecBuilder.fill(0, 0);
                /** Standard deviaiton in the encoder measurements for the module angle. */
                public final Vector<N1> ANGLE_ENC_STDEV = VecBuilder.fill(0);

                /** Qelm constant; measurement error tolerance. */
                public final Vector<N2> ANGLE_QELMS = VecBuilder.fill(0, 0);
                /** Relm constant; control effort tolerance. */
                public final Vector<N1> ANGLE_RELMS = VecBuilder.fill(0);

                /** kV Feed Forward constant for the speed */
                public final double SPEED_kV = 0;
                /** kA Feed Forward constant for the speed */
                public final double SPEED_kA = 0;

                /** Standard deviation in state predictions for the module speed. */
                public final double SPEED_STATE_STDEV = 0;
                /** Standard deviaiton in the encoder measurements for the module speed. */
                public final double SPEED_ENC_STDEV = 0;

                /** Qelm constant; measurement error tolerance. */
                public final double SPEED_QELMS = 0;
                /** Relm constant; control effort tolerance. */
                public final double SPEED_RELMS = 0;
            }
            public final class module4 extends ModuleConstants {
                /** Raw encoder reading when the module is at 0 degrees. */
                public static final double ZERO = 0.0;
                public static final int SPEED_ID = 7;
                public static final int ANGLE_ID = 8;
                public static final int ENC_ID = 14;

                /** kV Feed Forward constant for the angle */
                public final double ANGLE_kV = 0;
                /** kA Feed Forward constant for the angle */
                public final double ANGLE_kA = 0;

                /** Standard deviation in state predictions for the module angle. */
                public final Vector<N2> ANGLE_STATE_STDEV = VecBuilder.fill(0, 0);
                /** Standard deviaiton in the encoder measurements for the module angle. */
                public final Vector<N1> ANGLE_ENC_STDEV = VecBuilder.fill(0);

                /** Qelm constant; measurement error tolerance. */
                public final Vector<N2> ANGLE_QELMS = VecBuilder.fill(0, 0);
                /** Relm constant; control effort tolerance. */
                public final Vector<N1> ANGLE_RELMS = VecBuilder.fill(0);

                /** kV Feed Forward constant for the speed */
                public final double SPEED_kV = 0;
                /** kA Feed Forward constant for the speed */
                public final double SPEED_kA = 0;

                /** Standard deviation in state predictions for the module speed. */
                public final double SPEED_STATE_STDEV = 0;
                /** Standard deviaiton in the encoder measurements for the module speed. */
                public final double SPEED_ENC_STDEV = 0;

                /** Qelm constant; measurement error tolerance. */
                public final double SPEED_QELMS = 0;
                /** Relm constant; control effort tolerance. */
                public final double SPEED_RELMS = 0;
            }

            public final double XkP = 5.2;
            public final double XkI = 0.0;
            public final double XkD = 0.0;
            public final PIDController xPID = new PIDController(XkP, XkI, XkD);
        
            public final double YkP = 5.2;
            public final double YkI = 0.0;
            public final double YkD = 0.0;  
            public final PIDController yPID = new PIDController(YkP, YkI, YkD);

            public final double THETA_kP = -17.25;
            public final double THETA_kI = 0.0;
            public final double THETA_kD = -0.05;  
            public final ProfiledPIDController thetaPID = new ProfiledPIDController(THETA_kP, THETA_kI, THETA_kD, new Constraints(Math.PI * 2.0, Math.PI / 2.0));

            public final double AUTO_MAX_VELOCITY = 1.0; // Maximum velocity allowed in the drivetrain (Meters per Second).
            public final double AUTO_MAX_ACCEL = 3.0; // Maximum acceleration of the drivetrain in (Meters per Second Squared).
            public final double AUTO_MAX_CACCEL = 5.0; // Maximum centripital acceleration of the robot (Meters per Second Squared).

            public final ModuleConstants[] modules;
            private drive() {
                modules = new ModuleConstants[] {new module1(), new module2(), new module3(), new module4()};
            };
        }

        public static class catcher {

        }
        public static class intake {
            
        }
        public static class climber {
            
        }
        public static class shooter {
            
        }

    private static Constants instance = new Constants();
    public static drive drive = null;

    private Constants() {
        drive = new drive();
    };

    public static drive.ModuleConstants[] getModules() {
        return drive.modules;
    }
}

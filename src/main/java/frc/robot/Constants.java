package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.Kinematics;

public class Constants {

    /** Constants for the entire robot as a whole. */
    public final class robot {
        /** Meters */
        public static final double A_LENGTH = 0.59055;
        /** Meters */
        public static final double A_WIDTH = 0.48895;
        /** Meters */
        public final double A_CROSSLENGTH = Math.hypot(A_LENGTH, A_WIDTH);

        /** Counts / Revolution */
        public static final double FALCON_ENCODER_TICKS = 2048.0;
        /** Celsius */
        public static final double FALCON_MAX_TEMP = 50.0;
    }

    public static class drive {
        public static final double DRIVE_GEARING = 6.92; // Gear ratio of the drive motor.
        /** Meters */
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4); //Diameter of the drive wheels.
        /** Meters */
        public static final double WHEEL_CIRCUMFRENCE = Math.PI * WHEEL_DIAMETER; // Circumfrence of the drive wheels.
        /** Rotations / Meter */
        public static final double DRIVE_ROTATIONS_PER_METER = 1.0 / WHEEL_CIRCUMFRENCE; // Rotations per meter of the drive wheels.
        /** Counts / Rotation */
        public static final double DRIVE_COUNTS_PER_ROTATION = DRIVE_GEARING * robot.FALCON_ENCODER_TICKS; // Encoder counts per revolution of the drive wheel.
        /** Counts / Meter*/
        public static final double DRIVE_COUNTS_PER_METER = DRIVE_ROTATIONS_PER_METER * DRIVE_COUNTS_PER_ROTATION; // Encoder ticks per meter of the drive wheels.

        public static final double ANGLE_GEARING = 11.57;
        /** Counts / Degree */
        public static final double ANGLE_TICKS_PER_DEGREE = (ANGLE_GEARING * robot.FALCON_ENCODER_TICKS) / 360.0;

        /** Volts */
        public static final double MAX_VOLTS = 12.0; // Maximum voltage allowed in the drivetrain.
        /** Meters / Second */
        public static final double MAX_VELOCITY = 10.0; // Maximum velocity allowed in the drivetrain.
        /** Meters / Second^2 */
        public static final double MAX_ACCEL = 20.0; // Maximum acceleration of the drivetrain.
        /** Meters / Second^2 */
        public static final double MAX_CACCEL = 8.0; // Maximum centripital acceleration of the robot.
        /** Radians / Second */
        public static final double MAX_RADIANS = 3.0 * Math.PI; // Maximum rotational velocity.

        // Put together swerve module positions relative to the center of the robot.
        public static final Translation2d FrontLeftLocation = new Translation2d(-(Constants.robot.A_WIDTH / 2), -(Constants.robot.A_LENGTH / 2));
        public static final Translation2d FrontRightLocation = new Translation2d(-(Constants.robot.A_WIDTH / 2), (Constants.robot.A_LENGTH / 2));
        public static final Translation2d BackLeftLocation = new Translation2d((Constants.robot.A_WIDTH / 2), -(Constants.robot.A_LENGTH / 2));
        public static final Translation2d BackRightLocation = new Translation2d((Constants.robot.A_WIDTH / 2), (Constants.robot.A_LENGTH / 2));
        public static final Kinematics KINEMATICS = new Kinematics(FrontLeftLocation, FrontRightLocation, BackLeftLocation, BackRightLocation);

        public static final class modules {
            public static final double M1_ZERO = 45.263672;
            public static final double M2_ZERO = -122.958984;
            public static final double M3_ZERO = -160.927734;
            public static final double M4_ZERO = -48.164062;
        }

        public static final class anglemotor {
            public static final double kP = -1e-2;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
        }

        public static final class speedmotor {
            public static final double kP = 19e-2;
            public static final double kI = 0.0;
            public static final double kD = 1e-1;
        }

        public static final class xPID {
            public static final double kP = 5.2;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final PIDController xPID = new PIDController(kP, kI, kD);
        }
        
        public static final class yPID {
            public static final double kP = 5.2;
            public static final double kI = 0.0;
            public static final double kD = 0.0;  
            public static final PIDController yPID = new PIDController(kP, kI, kD);
        }

        public static final class thetaPID {
            public static final double kP = -17.25;
            public static final double kI = 0.0;
            public static final double kD = -0.05;  
            public static final ProfiledPIDController thetaPID = new ProfiledPIDController(kP, kI, kD, new Constraints(Math.PI * 2.0, Math.PI / 2.0));
        }

        public static final class auto {
            /** Meters / Second */
            public static final double MAX_VELOCITY = 1.0; // Maximum velocity allowed in the drivetrain.
            /** Meters / Second^2 */
            public static final double MAX_ACCEL = 3.0; // Maximum acceleration of the drivetrain.
            /** Meters / Second^2 */
            public static final double MAX_CACCEL = 5.0; // Maximum centripital acceleration of the robot.

            public static final TrajectoryConfig CONFIG = new TrajectoryConfig(MAX_VELOCITY, MAX_ACCEL)
                .addConstraint(new CentripetalAccelerationConstraint(MAX_CACCEL)); 
        }
    }

    public static final class shooter {
        public static final class upper {
            public static double kV = 0.10964;
            public static double kA = 0.0148;

            public static final double GEARING = 1.0;
            public static final double RAMP = 2.0;

            public static double SYSTEM_STDEV = 99999e8;
            public static double ENC_STDEV = 0.001e-4;

            public static final double QELMS = 80.0;
            public static final double RELMS = 5.0;

            public static final double RADIUS = Units.inchesToMeters(3.0);
            public static final double CIRCUMFRENCE = RADIUS * 2.0 * Math.PI;
            public static final int MOTOR_ID = 10;
        }
        public static final class lower {
            public static final double GEARING = 1.5;
            public static final double RAMP = 2.0;

            public static double kV = 0.16214;
            public static double kA = 0.0089196;

            public static double SYSTEM_STDEV = 99999e8;
            public static double ENC_STDEV = 0.001e-4;

            public static final double QELMS = 80.0;
            public static final double RELMS = 5.0;

            public static final double RADIUS = Units.inchesToMeters(2.0);
            public static final double CIRCUMFRENCE = RADIUS * 2.0 * Math.PI;
            
            public static final int MOTOR_ID = 9;
        }
    }
    
    public static final class field {

        public static final double GOAL_HEIGHT = 2.64;

    }

    public static final class turret {
            public static final double kP = 0.03;
            public static final double kI = 0.0;
            public static final double kD = 0.00002;

            public static final double MIN_ANGLE = -55.0;
            public static final double MAX_ANGLE = 161.0;

            public static final double GEAR_RATIO = 20 * 14;
            public static final double ENCODER_TO_DEGREES = 360 / GEAR_RATIO;

            public static final double ANGLE_TOLERANCE = 0.0;
    
        
        public static final double LIMELIGHT_HEIGHT = 0.6477;
        public static final double MOUNTING_ANGLE = 19.7;

    }

    public static final class intake{

        public static final int MOTOR_ID = 17;
        public static final double RAMP_RATE = 1.5;

    };

    public static final class climber {
        public static final int MOTOR_A_ID = 16;
        public static final int MOTOR_B_ID = 15;
        public static final int BRAKE_A_ID = 5;
        public static final int BRAKE_B_ID = 4;
        public static final int SOLENOID_A_ID = 2;
        public static final int SOLENOID_B_ID = 3;

        /** Meters */
        public static final double PULL_POSITION = 0.0;
        /** Meters */
        public static final double ACTIVE_TOLERANCE = 0.03;
        /** Meters */
        public static final double PUSH_POSITION = Units.inchesToMeters(12.0 * 4.0 + 9.0);
        /** Meters */
        public static final double MAX_HEIGHT = Units.inchesToMeters(12.0 * 4.0 + 11.0);
        /** Meters / Count */
        public static final double METERS_PER_COUNT = Units.inchesToMeters(Math.PI);
        /** Meters / Second */
        public static final double MAX_VELOCITY = 0.25;
        /** Meters / Second^2 */
        public static final double MAX_ACCEL = 1.0;

        public static double kP = 2.0;
        public static double kI = 0.0;
        public static double kD = 0.0;

    }
}

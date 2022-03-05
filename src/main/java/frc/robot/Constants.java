package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.util.Units;

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
        public static final double MAX_VELOCITY = 5.0; // Maximum velocity allowed in the drivetrain.
        /** Meters / Second^2 */
        public static final double MAX_ACCEL = 20.0; // Maximum acceleration of the drivetrain.
        /** Meters / Second^2 */
        public static final double MAX_CACCEL = 16.0; // Maximum centripital acceleration of the robot.
        /** Radians / Second */
        public static final double MAX_RADIANS = 4.0 * Math.PI; // Maximum rotational velocity.

        // Put together swerve module positions relative to the center of the robot.
        public static final Translation2d FrontLeftLocation = new Translation2d(-(Constants.robot.A_WIDTH / 2), -(Constants.robot.A_LENGTH / 2));
        public static final Translation2d FrontRightLocation = new Translation2d(-(Constants.robot.A_WIDTH / 2), (Constants.robot.A_LENGTH / 2));
        public static final Translation2d BackLeftLocation = new Translation2d((Constants.robot.A_WIDTH / 2), -(Constants.robot.A_LENGTH / 2));
        public static final Translation2d BackRightLocation = new Translation2d((Constants.robot.A_WIDTH / 2), (Constants.robot.A_LENGTH / 2));

        public static final class modules {
            public static final double M1_ZERO = 175.95703125;
            public static final double M2_ZERO = 120.498046875;
            public static final double M3_ZERO = -97.03125;
            public static final double M4_ZERO = -164.794921875;
        }

        public static final class anglemotor {
            public static final double kP = -9e-3;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
        }

        public static final class speedmotor {
            public static final double kP = 25e-2;
            public static final double kI = 0.0;
            public static final double kD = 1e-1;
        }

        public static final class xPID {
            public static final double kP = 4.8;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final PIDController xPID = new PIDController(kP, kI, kD);
        }
        
        public static final class yPID {
            public static final double kP = 4.8;
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
            public static final double MAX_VELOCITY = 3.0; // Maximum velocity allowed in the drivetrain.
            /** Meters / Second^2 */
            public static final double MAX_ACCEL = 3.25; // Maximum acceleration of the drivetrain.
            /** Meters / Second^2 */
            public static final double MAX_CACCEL = 2.5; // Maximum centripital acceleration of the robot.

            public static final TrajectoryConfig CONFIG = new TrajectoryConfig(MAX_VELOCITY, MAX_ACCEL)
                .addConstraint(new CentripetalAccelerationConstraint(MAX_CACCEL)); 
        }
    }

    public static final class shooter {
        public static final int FOLLOWER_ID = 13;
        public static final int LEADER_ID = 14;

        public static double kS = 0.72969;
        public static double kV = 0.07986;
        public static double kA = 0.0058329;

        public static final double GEARING = 24.0/32.0;
        public static final double RAMP = 2.5;

        public static final double RADIUS = Units.inchesToMeters(4.0);
        public static final double CIRCUMFRENCE = RADIUS * 2.0 * Math.PI;

        public static final double kP = 0.23619;
        public static final double kD = 5e-4;
        public static final double MAX_ACCEL = 1000.0;
        public static final double MAX_JERK = 5000.0;

        public static final class hood {
            public static final int MOTOR_ID = 12;
            public static final int ENCODER_ID = 27;

            public static final double kP = 0.45;
            public static final double kI = 0.0;
            public static final double kD = 0.01;
            public static final double kCos = 0.53;

            public static final double GEARING = 7.0 * 7.0 * (40.0/16.0);
            public static final double CONVERSION = 30.0 / 10.74;//360.0 / (GEARING);
            public static final double OFFSET = 9.8;
            
            public static final double MAX = 45.0;
            public static final double MIN = 0.0;
        }
    }
    
    public static final class field {
        public static final double GOAL_HEIGHT = 2.64;
    }

    public static final class turret {
            public static final double kP = 0.03;
            public static final double kI = 0.0;
            public static final double kD = 0.001;

            public static final double MIN_ANGLE = -55.0;
            public static final double MAX_ANGLE = 161.0;

            public static final double GEAR_RATIO = 20.0 * 14.0;
            public static final double ENCODER_TO_DEGREES = 360.0 / GEAR_RATIO;

            public static final double ANGLE_TOLERANCE = 0.0;
    
            public static final int MOTOR_ID = 9;
        
        public static final double LIMELIGHT_HEIGHT = Units.inchesToMeters(43.6);
        public static final double MOUNTING_ANGLE = 20.0;

    }

    public static final class intake{

        public static final int MOTOR_ID = 17;
        public static final double RAMP_RATE = 1.5;
        public static final int SOLENOID_A_ID = 0;
        public static final int SOLENOID_B_ID = 1;

    };

    public static final class climber {
        public static final int MOTOR_A_ID = 16;
        public static final int MOTOR_B_ID = 15;
        public static final int ENCODER_A_ID = 25;
        public static final int ENCODER_B_ID = 26;
        public static final int BRAKE_A_ID = 4;
        public static final int BRAKE_B_ID = 5;

        public static final int LEFT_IN = 4;
        public static final int LEFT_OUT = 5;
        public static final int RIGHT_IN = 2;
        public static final int RIGHT_OUT = 3;
                ;
        public static final int SOLENOID_A_ID = 3;
        public static final int SOLENOID_B_ID = 2;

        /** Meters */
        public static final double PULL_POSITION = 0.0;
        /** Meters */
        public static final double ACTIVE_TOLERANCE = 0.0075;
        /** Meters */
        public static final double PUSH_POSITION = 0.97;
        /** Meters */
        public static final double MAX_HEIGHT = 0.97;
        /** Meters / Count */
        public static final double METERS_PER_COUNT = ((0.97 / 2898.105469) / 10.6) * (2048.0 / 360.0);
        /** Meters / Second */
        public static final double MAX_VELOCITY = 0.7;
        /** Meters / Second^2 */
        public static final double MAX_ACCEL = 1.8;
        public static final double PITCH_TOLERANCE = 2.5;

        public static double kP = 96.0;
        public static double kI = 20.0;
        public static double kD = 7.2;

    }

    public static final class indexer {
        public static final int INDEXER_ID = 10;
        public static final int TRIGGER_ID = 11;
        public static final int TOP_BB = 1;
        public static final int BOTTOM_BB = 0;
    }
}

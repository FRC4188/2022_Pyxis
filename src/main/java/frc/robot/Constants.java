package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.sensors.BallDetector;
import frc.robot.subsystems.sensors.ColorSensor;
import frc.robot.subsystems.sensors.Limelight;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.utils.DoubleSolenoid;
import frc.robot.utils.motors.CSPMotor;
import frc.robot.utils.motors.CSP_CANSparkMax;
import frc.robot.utils.motors.CSP_Falcon;

public class Constants {

    public static final class devices {
        public static final CSPMotor turretMotor = new CSP_Falcon(9);
        public static final CSPMotor preshooterMotor = new CSP_CANSparkMax(11);
        public static final CSPMotor shooterLeader = new CSP_Falcon(14);
        public static final CSPMotor shooterFollower = new CSP_Falcon(13);
        public static final CSPMotor intakeMotor = new CSP_CANSparkMax(17);
        public static final CSPMotor indexerMotor = new CSP_Falcon(10, "Pyxis CANivore");
        public static final CSPMotor hoodMotor = new CSP_CANSparkMax(12);
        public static final CSPMotor climberA = new CSP_Falcon(16);
        public static final CSPMotor climberB = new CSP_Falcon(15, "Pyxis CANivore");

        public static final CANCoder encoderA = new CANCoder(25);
        public static final CANCoder encoderB = new CANCoder(26, "Pyxis CANivore");

        public static final Pigeon pigeon = new Pigeon(30);
        public static final Limelight limelight = new Limelight("limelight");
        public static final BallDetector ballDetector = new BallDetector("Ball Detector");
        public static final ColorSensor colorSensor = new ColorSensor(I2C.Port.kOnboard);
        public static final DigitalInput top = new DigitalInput(1);
        public static final DigitalInput bot = new DigitalInput(0);

        public static final DoubleSolenoid climberPiston = new DoubleSolenoid(3, 2);
        public static final DoubleSolenoid intakePiston = new DoubleSolenoid(0, 1);
    }

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
            public static final double M1_ZERO = 171.38671875;
            public static final double M2_ZERO = 119.53125;
            public static final double M3_ZERO = -156.357421875;
            public static final double M4_ZERO = 179.736328125;
        }

        public static final class anglemotor {
            public static final double kP = -6e-3;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
        }

        public static final class speedmotor {
            public static final double kP = 25e-2;
            public static final double kI = 0.0;
            public static final double kD = 1e-1;
        }

        public static final class xPID {
            public static final double kP = 3.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;//0.052;
            public static final PIDController xPID = new PIDController(kP, kI, kD);
        }
        
        public static final class yPID {
            public static final double kP = 3.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;//0.052;  
            public static final PIDController yPID = new PIDController(kP, kI, kD);
        }

        public static final class thetaPID {
            public static final double kP = -5.25;//-8.25;
            public static final double kI = 0.0;
            public static final double kD = 0.0;//-0.25;  
            public static final ProfiledPIDController thetaPID = new ProfiledPIDController(kP, kI, kD, new Constraints(Math.PI * 2.0, Math.PI / 2.0));
        }

        public static final class auto {
            /** Meters / Second */
            public static final double MAX_VELOCITY = 1.0; // Maximum velocity allowed in the drivetrain.
            /** Meters / Second^2 */
            public static final double MAX_ACCEL = 1.75; // Maximum acceleration of the drivetrain.
            /** Meters / Second^2 */
            public static final double MAX_CACCEL = 1.75; // Maximum centripital acceleration of the robot.

            public static final TrajectoryConfig CONFIG = new TrajectoryConfig(MAX_VELOCITY, MAX_ACCEL)
                .addConstraint(new CentripetalAccelerationConstraint(MAX_CACCEL)); 
        }
    }

    public static final class shooter {
        public static final double ALPHA = 0.98;

        public static double kS = 0.7391;
        public static double kV = 0.09001;
        public static double kA = 0.0072442;

        public static final double GEARING = 1;
        public static final double RAMP = 2.5;

        public static final double RADIUS = Units.inchesToMeters(4.0);
        public static final double CIRCUMFRENCE = RADIUS * 2.0 * Math.PI;

        public static final double kP = 0.2;
        public static final double kI = 0.0;
        public static final double kD = 5e-4;
        public static final double MAX_ACCEL = 1000.0;
        public static final double MAX_JERK = 5000.0;

        public static final class hood {
            public static final double BETA = 1.175;

            public static final double kP = 1.0;
            public static final double kI = 0.0;
            public static final double kD = 0.025;
            public static final double kCos = 0.53;

            public static final double GEARING = 5.0 * 5.0 * 7.0 * (40.0/16.0);
            public static final double CONVERSION = 360.0 / (GEARING);
            public static final double OFFSET = 8.6;
            
            public static final double MAX = 32.0;
            public static final double MIN = 0.0;

        }
    }
    
    public static final class field {
        public static final double GOAL_HEIGHT = 2.64;
    }

    public static final class turret {
            public static final double TkP = 0.35;
            public static final double TkI = 0.0;
            public static final double TkD = 0.0525;

            public static final double PkP = 0.5;
            public static final double PkI = 0.0;
            public static final double PkD = 0.0;
            public static final double MAX_VEL = 720.0;
            public static final double MAX_ACCEL = 1440.0;

            public static final double MIN_ANGLE = -400.0;
            public static final double MAX_ANGLE = 55.0;

            public static final double GEAR_RATIO = 3.0 * 3.0 * 14.0;
            public static final double ENCODER_TO_DEGREES = 360.0 / GEAR_RATIO;

            public static final double ANGLE_TOLERANCE = 2.0;
            
        public static final double LIMELIGHT_HEIGHT = Units.inchesToMeters(43.6);
        public static final double MOUNTING_ANGLE = 27.5;

    }

    public static final class intake{

        public static final double RAMP_RATE = 1.5;
        public static final int SOLENOID_A_ID = 0;
        public static final int SOLENOID_B_ID = 1;

    };

    public static final class climber {
        /** Meters */
        public static final double PULL_POSITION = 0.0;
        /** Meters */
        public static final double ACTIVE_TOLERANCE = 0.02;
        /** Meters */
        public static final double PUSH_POSITION = 0.878675;
        /** Meters */
        public static final double MAX_HEIGHT = 0.878675;
        /** Meters / Count */
        public static final double METERS_PER_COUNT_B = 0.828675 / 136.28076171875;
        public static final double METERS_PER_COUNT_A = 0.828675 / 134.2822265625;

        /** Meters / Second */
        public static final double MAX_VELOCITY = 0.75;
        /** Meters / Second^2 */
        public static final double MAX_ACCEL = 1.5;
        public static final double PITCH_TOLERANCE = 2.5;

        public static double kP = 80.0;
        public static double kI = 20.0;
        public static double kD = 6.2;

    }

    public static class sensors {
        public static final double CAMERA_FOV = 29.4;
        public static final double CAMERA_WIDTH = 320;
        public static final double DISTANCE_SCALAR = 0.0;
    }
}

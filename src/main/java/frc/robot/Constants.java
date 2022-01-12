package frc.robot;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.EllipticalRegionConstraint;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.util.Units;

/**
 * A class storing static variables holding constants.
 *
 * <p>Create constants here even if they will only be used once in the code; the idea is to have a
 * single tuning class.
 */
public final class Constants {

  /** Robot-wide constants. */
  public final class robot {

    /** Minimum available battery power before things get really bad (Volts). */
    public static final double MIN_VOLTS = 6.0;
    /** Battery power where we start getting a little bit worried (Volts). */
    public static final double MID_VOLTS = 8.0;

    /** Axel length (Meters). */
    public static final double A_LENGTH = 0.59055;
    /** Axel width (Meters). */
    public static final double A_WIDTH = 0.48895;
    /** Distance from one axel to the diagonal axel (Meters). */
    public final double A_CROSSLENGTH = Math.sqrt(Math.pow(A_LENGTH, 2.0) + Math.pow(A_WIDTH, 2.0));

    /** Maximum preferred temperature of a neo-550 motor (Celsius). */
    public static final double FIVEFIFTY_MAX_TEMP = 50.0;
    /** Maximum preferred temperature of a neo motor (Celsius). */
    public static final double NEO_MAX_TEMP = 50.0;
    /** Counts per revolution of the Falcon 500 motor. */
    public static final double FALCON_ENCODER_TICKS = 2048.0;
    /** Maximum preferred temperature of Falcon-500 motor (Celsius). */
    public static final double FALCON_MAX_TEMP = 50.0;
    /** Maximum velocity of a Falcon-500 motor with no load (RPM). */
    public static final double FALCON_MAX_VEL = 6380.0;
  }

  /** Drivetrain specific constants. */
  public static class drive {
    /** Gear ratio of the drive motor. */
    public static final double DRIVE_GEARING = 6.92;
    /** Diameter of the drive wheels (Meters). */
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    /** Circumfrence of the drive wheels (Meters). */
    public static final double WHEEL_CIRCUMFRENCE = Math.PI * WHEEL_DIAMETER;
    /** Rotations over distance of the drive wheels (Rotations per Meter). */
    public static final double DRIVE_ROTATIONS_PER_METER = 1.0 / WHEEL_CIRCUMFRENCE;
    /** Encoder counts per revolution of the drive wheel. */
    public static final double DRIVE_COUNTS_PER_ROTATION =
        DRIVE_GEARING * robot.FALCON_ENCODER_TICKS;
    /** Encoder ticks over distance of the drive wheels (Counts per Meter). */
    public static final double DRIVE_COUNTS_PER_METER =
        DRIVE_ROTATIONS_PER_METER * DRIVE_COUNTS_PER_ROTATION;

    /** Gear ratio of the angle motors. */
    public static final double ANGLE_GEARING = 11.57;
    /** Encoder counts over rotation of the angle motors (Counts per Degree). */
    public static final double ANGLE_TICKS_PER_DEGREE =
        (ANGLE_GEARING * robot.FALCON_ENCODER_TICKS) / 360.0;

    /** Maximum voltage allowed in the drivetrain. */
    public static final double MAX_VOLTS = 12.0;
    /** Maximum velocity allowed in the drivetrain (Meters per Second). */
    public static final double MAX_VELOCITY = 5.0;
    /** Maximum acceleration of the drivetrain in (Meters per Second Squared). */
    public static final double MAX_ACCEL = 20.0;
    /** Maximum centripital acceleration of the robot (Meters per Second Squared). */
    public static final double MAX_CACCEL = 8.0;
    /** Maximum rotational velocity (Radians per Second). */
    public static final double MAX_RADIANS = 3.0 * Math.PI;

    /** Position of the Front-Left module relative to the center of the robot (Meters). */
    public static final Translation2d FrontLeftLocation =
        new Translation2d(-(Constants.robot.A_WIDTH / 2), -(Constants.robot.A_LENGTH / 2));
    /** Position of the Front-Right module relative to the center of the robot (Meters). */
    public static final Translation2d FrontRightLocation =
        new Translation2d(-(Constants.robot.A_WIDTH / 2), (Constants.robot.A_LENGTH / 2));
    /** Position of the Back-Left module relative to the center of the robot (Meters). */
    public static final Translation2d BackLeftLocation =
        new Translation2d((Constants.robot.A_WIDTH / 2), -(Constants.robot.A_LENGTH / 2));
    /** Position of the Back-Right module relative to the center of the robot (Meters). */
    public static final Translation2d BackRightLocation =
        new Translation2d((Constants.robot.A_WIDTH / 2), (Constants.robot.A_LENGTH / 2));

    /** Kinematics object for the swerve drivetrain. */
    public static final SwerveDriveKinematics KINEMATICS =
        new SwerveDriveKinematics(
            FrontLeftLocation, FrontRightLocation, BackLeftLocation, BackRightLocation);

    /** Magnetic zero of the M1 module (Degrees). */
    public static final double M1_ZERO = 37.792969;
    /** Magnetic zero of the M1 module (Degrees). */
    public static final double M2_ZERO = 177.187500;
    /** Magnetic zero of the M1 module (Degrees). */
    public static final double M3_ZERO = -42.275391;
    /** Magnetic zero of the M1 module (Degrees). */
    public static final double M4_ZERO = 128.583984;

    /** PID Constants for the angle motors. */
    public static final class anglemotor {
      /** Proportional term (kP). */
      public static final double kP = -1e-2;
      /** Integral term (kI). */
      public static final double kI = 0.0;
      /** Derivative term (kD). */
      public static final double kD = 0.0;
    }

    /** PID Constants for the speed motors. */
    public static final class speedmotor {
      /** Proportional term (kP). */
      public static final double kP = 24e-2;
      /** Integral term (kI). */
      public static final double kI = 0.0;
      /** Derivative term (kD). */
      public static final double kD = 1e-1;
    }

    /** X Axis PID Constants (Autonomous). */
    public static final class xPID {
      /** Proportional term (kP). */
      public static final double kP = 52e-1;
      /** Integral term (kI). */
      public static final double kI = 0.0;
      /** Derivative term (kD). */
      public static final double kD = 0.0;
      /** PID Controller object. */
      public static final PIDController xPID = new PIDController(kP, kI, kD);
    }

    /** Y Axis PID Constants (Autonomous). */
    public static final class yPID {
      /** Proportional term (kP). */
      public static final double kP = 52e-1;
      /** Integral term (kI). */
      public static final double kI = 0.0;
      /** Derivative term (kD). */
      public static final double kD = 0.0;
      /** PID Controller object. */
      public static final PIDController yPID = new PIDController(kP, kI, kD);
    }

    /** Heading PID constants (Autonomous). */
    public static class thetaPID {
      /** Proportional term (kP). */
      public static final double kP = 6e-2;
      /** Integral term (kI). */
      public static final double kI = 0.0;
      /** Derivative term (kD). */
      public static final double kD = 15e-3;
      /** PID Controller object. */
      public static final ProfiledPIDController thetaPID =
          new ProfiledPIDController(kP, kI, kD, new Constraints(270.0, 90.0));
    }

    /** Autonomous configuration constants. */
    public static final class auto {
      /** Maximum velocity allowed in the drivetrain (Meters per Second). */
      public static final double MAX_VELOCITY = 2.5;
      /** Maximum acceleration of the drivetrain in (Meters per Second Squared). */
      public static final double MAX_ACCEL = 1.5;
      /** Maximum velocity of the robot when driving over the steel bars. */
      public static final double BAR_VELOCITY = 0.5;
      /** {@link TrajectoryConfig} object to control trajectory generation. */
      public static final TrajectoryConfig CONFIG =
          new TrajectoryConfig(MAX_VELOCITY, MAX_ACCEL)
              .addConstraint(new CentripetalAccelerationConstraint(MAX_ACCEL))
              .addConstraint(
                  new EllipticalRegionConstraint(
                      new Translation2d(3.518223 * 1.74759405074, 2.317102 * 1.79571303587),
                      0.3,
                      0.7,
                      Rotation2d.fromDegrees(22.341197),
                      new MaxVelocityConstraint(BAR_VELOCITY)))
              .addConstraint(new SwerveDriveKinematicsConstraint(KINEMATICS, MAX_VELOCITY));
    }
  }

  /** Field measurement constants. */
  public static final class field {
    /** Height of the goal from the ground (Meters). */
    public static final double GOAL_HEIGHT = 2.4954282231;
    /** Height of the port from top to bottom (Feet). */
    public static final double PORT_HEIGHT = 2.5;
    /** Depth of the 3 point goal inside the 2 point goal (Meters). */
    public static final double THREE_POINT_DEPTH = Units.feetToMeters(2.0 + (5.25 / 12.0));
    /** Length of one side of the port (Meters). */
    public static final double PORT_SIDE_LENGTH =
        Units.feetToMeters(PORT_HEIGHT / (2.0 * Math.sin(Math.toRadians(60.0))));
    /** Limit of the offset targeting before hitting the outside of the port (Degrees). */
    public static final double OFFSET_LIMIT =
        Math.toDegrees(Math.atan(PORT_SIDE_LENGTH / (THREE_POINT_DEPTH * 2.0)));
    /** Diameter of the power cell (Meters). */
    public static final double POWER_CELL_DIAMETER = Units.feetToMeters(7.0 / 12.0);
    /** Diameter of the 3-point inner port (Meters). */
    public static final double INNER_PORT_DIAMETER = Units.feetToMeters(13.0 / 12.0);
  }

  /** Turret specific constants. */
  public static final class turret {
    /** Height fromt the ground of the limelight (Meters). */
    public static final double LIMELIGHT_HEIGHT = 0.67;
    /** Angle from horizontal the Limelight is mounted at (Degrees). */
    public static final double MOUNTING_ANGLE = 19.7;

    /** Gear ratio of the turret motor */
    public static final double GEAR_RATIO = 300.0;

    public static final double ENCODER_TO_DEGREES = 360.0 / GEAR_RATIO;

    /** Proportional term (kP). */
    public static final double kP = 0.03;
    /** Integral term (kI). */
    public static final double kI = 0.0;
    /** Derivative term (kD). */
    public static final double kD = 0.003;

    /** Positional tolerance for aiming (Degrees). */
    public static final double POS_TOLERANCE = 2.0;
    /** Velocity tolerance for aiming (Degrees). */
    public static final double VEL_TOLERANCE = 2.0;

    /** Farthest positive bound of the turret's roation (Degrees). */
    public static final double MAX_ANG = 360.0;
    /** Farthest negative bound of the turret's roation (Degrees). */
    public static final double MIN_ANG = -45.0;
  }

  /** Shooter specific constants. */
  public static final class shooter {
    /** Highest velocity of the shooter (RPM). */
    public static final double MAX_VELOCITY = 5750.0;
    /** Resting velocity of the shooter when not shooting (RPM). */
    public static final double IDLE_VEL = 0.0;
    /** Velocity tolerance to shoot (RPM). */
    public static final double SHOOTING_TOLERANCE = 250.0;

    /** Proportional term (kP). */
    public static final double kP = 0.325;
    /** Integral term (kI). */
    public static final double kI = 0.0;
    /** Derivative term (kD). */
    public static final double kD = 32.5;
    /** Arbitrary Feedforward term (kF). */
    public static final double kF = (1023.0 * 0.75) / 12780.0;

    /** Ramp rate of the shooter motors (Seconds from 0.0 to 1.0 power.) */
    public static final double RAMP_RATE = 1.0;

    /** Height of the shooter off of the ground (Meters). */
    public static final double SHOOTER_HEIGHT = Units.feetToMeters(2.0);
    /** Diameter of the shooter wheel (Meters). */
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
  }

  public static final class shootersysid {
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    
  }

  /** Intake specific constants. */
  public final class intake {
    /** Ramp rate of the intake motor (Seconds from 0.0 to 1.0 power.) */
    public static final double RAMP_RATE = 0.5;
  }

  /** Hopper specific constants. */
  public static final class hopper {
    /** Ramp rate of the hopper motor (Seconds from 0.0 to 1.0 power.) */
    public static final double RAMP_RATE = 0.5;
  }

  /** Climber specific constants. */
  public static final class climber {
    /** Gear Ratio of the climber. */
    private static final double GEAR_RATIO = (58.0 / 11.0) * (20.0 / 60.0);
    /** Maximum allowed height of the climber (Meters). */
    public static final double MAX_HEIGHT = 1.3;
    /** Multiply this by encoder ticks to get meters. */
    public static final double ENCODER_TO_REV = 1.0 / (GEAR_RATIO * robot.FALCON_ENCODER_TICKS);

    public static final double kP = 0.4;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double P = 0.8; // Proportional term in PDS controller
    public static final double D = 0.027; // Deviational term in PDS controller
    public static final double S = 0.413; // Squaring term in PDS controller
    /** Maximum allowed valocity of the climber (Meters per Second). */
    public static final double MAX_VELOCITY = 0.5;
    /** Ramp rate of the climber motors (Seconds from 0.0 to 1.0 power.) */
    public static final double RAMP_RATE = 0.5;
  }

  public static final class controller {
    public static final double DEADBAND = 0.15;
    public static final double TRIGGER_THREADSHOLD = 0.6;
  }
}

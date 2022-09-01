// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.ColorSensorV3;

import csplib.motors.CSP_SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.drivetrain.SwerveModule;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.subsystems.shooter.Flywheel;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class Field {
        public static final double GOAL_HEIGHT = 0.0;
    }

    public static final class Robot {
        public static final double FALCON_CPM = 2048.0;
    }

    public static final class Drivetrain {
        public static final class Components {
            public static final SwerveModule FL_MODULE = new SwerveModule(4, 3, 22, 0.0);
            public static final SwerveModule FR_MODULE = new SwerveModule(2, 1, 21, 0.0);
            public static final SwerveModule BL_MODULE = new SwerveModule(6, 5, 23, "Pyxis CANivore", 0.0);
            public static final SwerveModule BR_MODULE = new SwerveModule(8, 7, 24, "Pyxis CANivore", 0.0);
        }

        public static final class Controllers {
            public static final PIDController X_CONTROLLER = new PIDController(0.0, 0.0, 0.0);
            public static final PIDController Y_CONTROLLER = new PIDController(0.0, 0.0, 0.0);
            public static final ProfiledPIDController ROT_CONTROLLER = new ProfiledPIDController(0.0, 0.0, 0.0, new Constraints(0.0, 0.0));
        }

        public static final class Locations {
            public static final Translation2d FL_LOCATION = new Translation2d(0.381, 0.381);
            public static final Translation2d FR_LOCATION = new Translation2d(0.381, -0.381);
            public static final Translation2d BL_LOCATION = new Translation2d(-0.381, 0.381);
            public static final Translation2d BR_LOCATION = new Translation2d(-0.381, -0.381);
        }

        public static final double SPEED_kP = 0.0;
        public static final double SPEED_kI = 0.0;
        public static final double SPEED_kD = 0.0;
        public static final double SPEED_kF = 0.0;

        public static final double ANGLE_kP = 0.0;
        public static final double ANGLE_kI = 0.0;
        public static final double ANGLE_kD = 0.0;
        public static final double ANGLE_kF = 0.0;

        public static final double MAX_SPEED = 0.0;
        public static final double MAX_ROT_SPEED = 0.0;

        public static final TrajectoryConfig AUTO_CONFIG = new TrajectoryConfig(3.0, 1.0);
    }

    public static final class Shooter {
        public static final Flywheel FLYWHEEL = new Flywheel(14, 13);
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double ACCURACY = 0.0;
        public static final double QELMS = 0.0;
    }

    public static final class Sensors {
        public static final class Devices {
            public static final ColorSensorV3 SENSOR = new ColorSensorV3(I2C.Port.kOnboard);
            public static final Pigeon IMU = new Pigeon(30);
        }
        public static final Color BLUE = new Color(0.1619, 0.3984, 0.4399);
        public static final Color RED = new Color(0.48, 0.36, 0.16);

        public static final double CAMERA_FOV = 29.4;
        public static final double CAMERA_WIDTH = 320;
        public static final double DISTANCE_SCALAR = 0.0;
    }

    public static final class Turret {
        public static final CSP_SparkMax MOTOR = new CSP_SparkMax(9);

        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double ACCURACY = 0.0;
        public static final double P_ERROR = 0.0;
        public static final double V_ERROR = 0.0;

        public static final double LL_MOUNTING_ANGLE = 0.0;
        public static final double LL_HEIGHT = 0.0;
    }
}

package frc.robot;

public final class ModuleConstants {
    public static final double ZERO[] = {0.0, 0.0, 0.0, 0.0};
    public static final int SPEED_ID[] = {2, 4, 5, 8};
    public static final int ANGLE_ID[] = {1, 2, 3, 4};
    public static final int ENC_ID[] = {21, 22, 23, 24};

    public static final double ANGLE_MAX_VEL = 1800.0;
    public static final double ANGLE_MAX_ACCEL = 3600.0;
    public static final double SPEED_MAX_ACCEL = 4.0;
    public static final double SPEED_MAX_JERK = 12.0;

    public static final double ANGLE_kP = 6e-3;
    public static final double ANGLE_kI = 0.0;
    public static final double ANGLE_kD = 0.0;
    public static final double ANGLE_kS = 0.79321;
    /** kV Feed Forward constant for the angle */
    public static final double ANGLE_kV = 1.1889e-2;

    public static final double SPEED_kP = 0.0;
    public static final double SPEED_kI = 0.0;
    public static final double SPEED_kD = 0.0;
    public static final double SPEED_kS = 0.0;
    /** kV Feed Forward constant for the speed */
    public static final double SPEED_kV = 0.1;
    /** kA Feed Forward constant for the speed */
    public static final double SPEED_kA = 0.1;
}

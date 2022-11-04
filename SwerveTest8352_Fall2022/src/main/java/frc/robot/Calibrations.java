package frc.robot;

public final class Calibrations {
    // Steer PID Values
    public static final double STEER_P = 0.1;
    public static final double STEER_I = 0;
    public static final double STEER_D = 0;

    // Wheel angle zeros
    public static int LEFT_FRONT_ZERO = 0;
    public static int LEFT_REAR_ZERO = 0;
    public static int RIGHT_FRONT_ZERO = 0;
    public static int RIGHT_REAR_ZERO = 0;

    // Dimensions
    public static final double WHEELBASE_LENGTH = 1.0; // From center of back wheel to center of front wheel, in feet
    public static final double WHEELBASE_WIDTH = 1.0; // From center of left wheel to center of right wheel, in feet
    public static final double CHASSIS_LENGTH = 1.0; // Entire chassis length, including bumpers, in feet
    public static final double CHASSIS_WIDTH = 1.0; // Entire chassis width, including bumpers, in feet

    // Chassis Speeds
    public static final double MAX_FORWARD_SPEED = 5.0; // feet per sec
    public static final double MAX_STRAFE_SPEED = 5.0; // feet per sec
    public static final double MAX_TURN_SPEED = 1.0; // rads per sec

    // Encoder Conversion
    public static final double STEER_ENCODER_RADIAN_PER_PULSE = 360; // 1 / (pulse_per_motor_rot * gearRatio * 2 * PI)
}

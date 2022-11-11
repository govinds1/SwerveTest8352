package frc.robot;

public final class RobotMap {
    // Controller
    public static final int DRIVE_CONTROLLER_PORT = 0;

    // Motor Controller IDs
    public static final int LEFT_FRONT_DRIVE_ID = 1;
    public static final int LEFT_FRONT_STEER_ID = 2;
    public static final int LEFT_REAR_DRIVE_ID = 3;
    public static final int LEFT_REAR_STEER_ID = 4;
    public static final int RIGHT_FRONT_DRIVE_ID = 5;
    public static final int RIGHT_FRONT_STEER_ID = 6;
    public static final int RIGHT_REAR_DRIVE_ID = 7;
    public static final int RIGHT_REAR_STEER_ID = 8;
    
    // Encoder Ports
    public static final int[] LEFT_FRONT_STEER_ENCODER_CHANNELS = {6, 7};
    public static final int[] LEFT_REAR_STEER_ENCODER_CHANNELS = {0, 1};
    public static final int[] RIGHT_FRONT_STEER_ENCODER_CHANNELS = {4, 5};
    public static final int[] RIGHT_REAR_STEER_ENCODER_CHANNELS = {2, 3};
}

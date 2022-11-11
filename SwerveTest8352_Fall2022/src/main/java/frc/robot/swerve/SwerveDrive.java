package frc.robot.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.robot.Calibrations;
import frc.robot.RobotMap;

public class SwerveDrive {
    WheelModule m_leftFront = new WheelModule(RobotMap.LEFT_FRONT_DRIVE_ID, RobotMap.LEFT_FRONT_STEER_ID, RobotMap.LEFT_FRONT_STEER_ENCODER_CHANNELS, Calibrations.LEFT_FRONT_ZERO);
    WheelModule m_leftRear = new WheelModule(RobotMap.LEFT_REAR_DRIVE_ID, RobotMap.LEFT_REAR_STEER_ID, RobotMap.LEFT_REAR_STEER_ENCODER_CHANNELS, Calibrations.LEFT_REAR_ZERO);
    WheelModule m_rightFront = new WheelModule(RobotMap.RIGHT_FRONT_DRIVE_ID, RobotMap.RIGHT_FRONT_STEER_ID, RobotMap.RIGHT_FRONT_STEER_ENCODER_CHANNELS, Calibrations.RIGHT_FRONT_ZERO);
    WheelModule m_rightRear = new WheelModule(RobotMap.RIGHT_REAR_DRIVE_ID, RobotMap.RIGHT_REAR_STEER_ID, RobotMap.RIGHT_REAR_STEER_ENCODER_CHANNELS, Calibrations.RIGHT_REAR_ZERO);

    Translation2d m_leftFrontLocation = new Translation2d(Calibrations.WHEELBASE_LENGTH / 2.0, Calibrations.WHEELBASE_WIDTH / 2.0);
    Translation2d m_leftRearLocation = new Translation2d(-Calibrations.WHEELBASE_LENGTH / 2.0, Calibrations.WHEELBASE_WIDTH / 2.0);
    Translation2d m_rightFrontLocation = new Translation2d(Calibrations.WHEELBASE_LENGTH / 2.0, -Calibrations.WHEELBASE_WIDTH / 2.0);
    Translation2d m_rightRearLocation = new Translation2d(-Calibrations.WHEELBASE_LENGTH / 2.0, -Calibrations.WHEELBASE_WIDTH / 2.0);

    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_leftFrontLocation, m_leftRearLocation, m_rightFrontLocation, m_rightRearLocation);
    ChassisSpeeds m_desiredSpeeds;
    ChassisSpeeds m_trueSpeeds;

    //ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
    boolean fieldRelative = false;

    public SwerveDrive() {

    }

    public void init() {
        m_leftFront.init();
        m_leftRear.init();
        m_rightFront.init();
        m_rightRear.init();
    }

    public void periodic() {
        // Wheel States should update first
        m_leftFront.periodic();
        m_leftRear.periodic();
        m_rightFront.periodic();
        m_rightRear.periodic();
      }

    // Takes direct input from controller axes, handle conversions to real units and proper robot coordinates in this function
    // Robot coords: 
    //      forward = +x
    //      right = -y
    //      turn = -omega (CCW (turning left) is positive)
    public void Drive(double forward, double right, double turn) {
        SetDesiredSpeeds(
            forward * Calibrations.MAX_FORWARD_SPEED,
            -right * Calibrations.MAX_STRAFE_SPEED,
            -turn * Calibrations.MAX_TURN_SPEED
        );
        
        /*m_leftRear.Drive(forward);
        m_leftFront.Drive(forward);
        m_rightRear.Drive(forward);
        m_rightFront.Drive(forward);
        m_leftRear.Steer(turn);
        m_leftFront.Steer(turn);
        m_rightRear.Steer(turn);
        m_rightFront.Steer(turn);*/
    }

    public void Drive(double forward, double right, double turn, boolean driveFieldRelative) {
        boolean fieldRelSaved = fieldRelative;
        SetFieldRelative(driveFieldRelative);
        Drive(forward, right, turn);
        SetFieldRelative(fieldRelSaved);
    }

    public void ResetSpeeds() {
        SetDesiredSpeeds(0, 0, 0);
    }

    // Zeroing options:
    //  Manually -> Turn all wheels to face forward and then calibrate (store the absolute encoder position or set encoder position to 0)
    //  Temporary Limit Switch -> Have a pin/limit switch so the wheel can spin until it hits it and stops (facing forward), then calibrate
    //  https://www.chiefdelphi.com/t/swerve-zeroing/181799
    public void CalibrateWheelsManually() {
        // Should only be called when all wheels are facing front (at the "zero" position)
        // Sets the angle encoder zeros to the current reading and stores them

        /*m_leftFront.CalibrateAngle();
        m_leftRear.CalibrateAngle();
        m_rightFront.CalibrateAngle();
        m_rightRear.CalibrateAngle();

        Calibrations.LEFT_FRONT_ZERO = m_leftFront.GetRawAngle();
        Calibrations.LEFT_REAR_ZERO = m_leftRear.GetRawAngle();
        Calibrations.RIGHT_FRONT_ZERO = m_rightFront.GetRawAngle();
        Calibrations.RIGHT_REAR_ZERO = m_rightRear.GetRawAngle();*/
        // print these to dashboard or save to file on disabled init
    }

    // vx and vy in feet per second, must be converted
    // omega in radians per second
    public void SetDesiredSpeeds(double vxFeetPerSecond, double vyFeetPerSecond, double omega) {
        double vxMpS = Units.feetToMeters(vxFeetPerSecond);
        double vyMpS = Units.feetToMeters(vyFeetPerSecond);
        if (fieldRelative) {
            //SetDesiredSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(vxMpS, vyMpS, omega, GetAngle()));
        } else {
            ChassisSpeeds newSpeeds = new ChassisSpeeds();
            newSpeeds.vxMetersPerSecond = vxMpS;
            newSpeeds.vyMetersPerSecond = vyMpS;
            newSpeeds.omegaRadiansPerSecond = omega;
            SetDesiredSpeeds(newSpeeds);
        }
    }

    public void SetDesiredSpeeds(ChassisSpeeds newSpeeds) {
        m_desiredSpeeds = newSpeeds;
        SetWheelStates();
    }

    public void SetWheelStates() {

        SwerveModuleState[] statesArray = m_kinematics.toSwerveModuleStates(m_desiredSpeeds);
        //SwerveDriveKinematics.desaturateWheelSpeeds(statesArray, Units.feetToMeters(Calibrations.MAX_FORWARD_SPEED + Calibrations.MAX_STRAFE_SPEED));
        m_leftFront.SetDesiredState(statesArray[0]);
        m_leftRear.SetDesiredState(statesArray[1]);
        m_rightFront.SetDesiredState(statesArray[2]);
        m_rightRear.SetDesiredState(statesArray[3]);
    }

    public void SetFieldRelative(boolean fieldRel) {
        fieldRelative = fieldRel;
    }

    // public void ResetGyro() {
    //     m_gyro.reset();
    // }

    // public Rotation2d GetAngle() {
    //     return m_gyro.getRotation2d();
    // }

    public ChassisSpeeds GetDesiredSpeeds() {
        return m_desiredSpeeds;
    }
}

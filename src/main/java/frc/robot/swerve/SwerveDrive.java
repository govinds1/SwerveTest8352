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
    boolean m_fieldRelative = false;

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
    // Field coords: 
    //      forward = +x
    //      right = -y
    //      turn = -omega (CCW (turning left) is positive)
    // Robot coords:
    //      forward = +y
    //      right = +x
    //      turn = +omega
    public void Drive(double forward, double right, double turn) {
        //Drive_WithMath(right, forward, turn);
        SetDesiredSpeeds(
            forward * Calibrations.MAX_FORWARD_SPEED,
            -right * Calibrations.MAX_STRAFE_SPEED,
            -turn * Calibrations.MAX_TURN_SPEED
        );
    }

    public void Drive(double forward, double right, double turn, boolean driveFieldRelative) {
        boolean fieldRelSaved = m_fieldRelative;
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
        if (m_fieldRelative) {
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
        m_fieldRelative = fieldRel;
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


    // Robot Coordinate system:
    // X is +right
    // Y is +forward
    // Rot is +CW (turning right)
    // vx, vy, and omega are in [-1, 1]
    public void Drive_WithMath(double vx, double vy, double omega) {
        // Factor in field relativity
        /*if (m_fieldRelative) {
            // vx and vy are relative to the field so we must convert to robot relative coordinates
            double currentHeading = GetHeading();
            // get robot relative components of vx and vy
            double vx_robotX = Math.cos(currentHeading) * vx;
            double vx_robotY = Math.sin(currentHeading) * vx;
            double vy_robotX = -Math.sin(currentHeading) * vy;
            double vy_robotY = Math.cos(currentHeading) * vy;

            // combine field relative components
            vx = vx_robotX + vy_robotX;
            vy = vx_robotY + vy_robotY;
        }*/

        // Derivation of Inverse Kinematics Swerve Drive: https://www.chiefdelphi.com/uploads/default/original/3X/8/c/8c0451987d09519712780ce18ce6755c21a0acc0.pdf
        // https://www.chiefdelphi.com/uploads/default/original/3X/e/f/ef10db45f7d65f6d4da874cd26db294c7ad469bb.pdf
        // Calculate Inverse Kinematics equations
        double A = vx - omega * (Calibrations.WHEELBASE_LENGTH / 2);
        double B = vx + omega * (Calibrations.WHEELBASE_LENGTH / 2);
        double C = vy - omega * (Calibrations.WHEELBASE_WIDTH / 2);
        double D = vy + omega * (Calibrations.WHEELBASE_WIDTH / 2);

        // Define Wheel velocities from IK equations
        double vx_fl = B;
        double vy_fl = D;
        double vx_fr = B;
        double vy_fr = C;
        double vx_rl = A;
        double vy_rl = D;
        double vx_rr = A;
        double vy_rr = C;


        // Calculate speeds and angles
        double v_fl = Math.sqrt(Math.pow(vx_fl, 2) + Math.pow(vy_fl, 2));
        double theta_fl = Math.atan2(vx_fl, vy_fl);
        double v_fr = Math.sqrt(Math.pow(vx_fr, 2) + Math.pow(vy_fr, 2));
        double theta_fr = Math.atan2(vx_fr, vy_fr);
        double v_rl = Math.sqrt(Math.pow(vx_rl, 2) + Math.pow(vy_rl, 2));
        double theta_rl = Math.atan2(vx_rl, vy_rl);
        double v_rr = Math.sqrt(Math.pow(vx_rr, 2) + Math.pow(vy_rr, 2));
        double theta_rr = Math.atan2(vx_rr, vy_rr);

        // Normalize speeds
        double maxV = Math.max(v_fl, Math.max(v_fr, Math.max(v_rl, v_rr)));
        if (maxV > 1) {
            v_fl = v_fl / maxV;
            v_fr = v_fr / maxV;
            v_rl = v_rl / maxV;
            v_rr = v_rr / maxV;
        }

        m_leftFront.SetDesiredState(v_fl, theta_fl);
        m_rightFront.SetDesiredState(v_fr, theta_fr);
        m_leftRear.SetDesiredState(v_rl, theta_rl);
        m_rightRear.SetDesiredState(v_rr, theta_rr);
    }
}

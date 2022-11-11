package frc.robot.swerve;

import java.io.Console;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Calibrations;

public class WheelModule {
    private VictorSPX m_driveMotor;
    private TalonSRX m_steerMotor;

    private PIDController m_steerPID;
    private Encoder m_steerEncoder;
    private int m_steerEncoderZero;

    SwerveModuleState m_desiredState;

    public WheelModule(int driveMotorID, int steerMotorID, int[] steerEncoderChannels, int steerEncoderZero) {
        m_driveMotor = new VictorSPX(driveMotorID);
        m_steerMotor = new TalonSRX(steerMotorID);

        m_steerEncoder = new Encoder(steerEncoderChannels[0], steerEncoderChannels[1]);
        m_steerEncoder.setDistancePerPulse(Calibrations.STEER_ENCODER_RADIAN_PER_PULSE); // PG Encoder
        m_steerEncoderZero = steerEncoderZero;
        m_steerPID = new PIDController(Calibrations.STEER_P, Calibrations.STEER_I, Calibrations.STEER_D);
        
    }

    public void init() {

    }
     
    public void periodic() {
        if (m_desiredState != null) {
            System.out.print(m_desiredState.speedMetersPerSecond);
            System.out.print(m_desiredState.angle.getDegrees());

            // Calculate angle PID output using desired state angle and set motor output
            Steer(m_steerPID.calculate(GetAngle().getRadians(), m_desiredState.angle.getRadians()));
            //Drive(m_desiredState.speedMetersPerSecond);
            Drive(Units.metersToFeet(m_desiredState.speedMetersPerSecond) / (Calibrations.MAX_FORWARD_SPEED + Calibrations.MAX_STRAFE_SPEED)); // NOT ACCURATE(probably)!!
        }
      }
  
      public void ResetState() {
          SetDesiredState(0, 0);
      }
  
      public void ResetAngle() {
          m_desiredState.angle = new Rotation2d(0);
      }
  
      public void CalibrateAngle() {
          // Call this function when the wheel faces forward, will set current encoder ticks as the zero variable
          //m_steerEncoderZero = m_steerEncoder.get(); // should be in raw ticks (without conversion) when using quadrature encoder
      }
  
      // Set the desired state of the swerve module -> setpoints from drive input
      public void SetDesiredState(SwerveModuleState newState) {
          m_desiredState = SwerveModuleState.optimize(newState, GetAngle());
      }
  
      public void SetDesiredState(double feetPerSec, double radians) {
          SetDesiredState(feetPerSec, new Rotation2d(radians));
      }
  
      public void SetDesiredState(double feetPerSec, Rotation2d radians) {
          SetDesiredState(new SwerveModuleState(feetPerSec, radians));
      }

      public void Drive(double power) {
          m_driveMotor.set(VictorSPXControlMode.PercentOutput, power);
      }

      public void Steer(double power) {
          m_steerMotor.set(TalonSRXControlMode.PercentOutput, power);
      }
  
      // Wheel angle of 0 (from a getter function) is considered to be facing forwards
      // Wheel azimuth in encoder pulses
      public int GetRawAngle() {
          return m_steerEncoder.get() - m_steerEncoderZero;
          //return 0;
      }
  
      // Wheel azimuth in radians
      public Rotation2d GetAngle() {
          return new Rotation2d(m_steerEncoder.getDistance() - (m_steerEncoderZero * m_steerEncoder.getDistancePerPulse()));
          //return new Rotation2d(0);
      }
  
      public SwerveModuleState GetDesiredState() {
          return m_desiredState;
      }

}

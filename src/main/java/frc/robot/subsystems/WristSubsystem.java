package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class WristSubsystem extends SubsystemBase {
    private final Servo m_rollServo; // Rev Smart Servo for roll
    private final SparkMax m_pitchMotor; // Neo 550 for pitch

    private SparkMaxConfig pitchMotorConfig; // Config for Neo 550
    private SparkClosedLoopController pitchPid; // PID controller for pitch
    private RelativeEncoder encoder; // Encoder for Neo 550

    private double targetRoll = 0; // Desired roll angle in degrees
    private double targetPitch = 0; // Desired pitch angle in degrees
    
    // Constants for pitch angle conversion
    private static final double GEAR_RATIO = 45.0; // 45:1 Reduction
    private static final double ENCODER_COUNTS_PER_MOTOR_REV = 42.0;
    private static final double DEGREES_PER_REV = 360.0;
    
    // Differential ratio between roll and pitch
    private final double wristDiffRatio = 1.0/3.0;
    
    // Minimum angle for compensation calculations
    private static final double MIN_COMPENSATION_ANGLE = 1.0;
    
    public WristSubsystem() {
        m_rollServo = new Servo(ArmConstants.kWristDiffServoPort);
        m_pitchMotor = new SparkMax(ArmConstants.kWristPitchMotorPort, MotorType.kBrushless);
        pitchPid = m_pitchMotor.getClosedLoopController();
        encoder = m_pitchMotor.getEncoder();
        pitchMotorConfig = new SparkMaxConfig();

        // Set brake mode
        pitchMotorConfig
            .inverted(false)
            .smartCurrentLimit(3) // Trying to minimize damage to wrist from powerful motor
            .idleMode(IdleMode.kBrake);
            
        // Set up encoders with conversion factor for degrees
        double positionConversionFactor = DEGREES_PER_REV / (ENCODER_COUNTS_PER_MOTOR_REV * GEAR_RATIO);
        double velocityConversionFactor = positionConversionFactor / 60.0; // Convert RPM to degrees/second
        
        pitchMotorConfig.encoder
            .positionConversionFactor(positionConversionFactor)
            .velocityConversionFactor(velocityConversionFactor);
            
        // Configure PID loop - adjusted gains for degree-based control
        pitchMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.01, 0.0, 0.0);  // Adjusted PID values for degree control
            
        // Set soft limits in degrees
        pitchMotorConfig.softLimit
            .forwardSoftLimit(7.0)  // Maximum 45 degrees down
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(-0.01) // Maximum 0 degrees up (relative encoder, should be 0?)
            .reverseSoftLimitEnabled(true);

        m_pitchMotor.configure(pitchMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setTargetAngles(double roll, double pitch) {
        // Clamp pitch angle to soft limits
        this.targetPitch = Math.max(Math.min(pitch, 7.0), 0.0);
        // Clamp roll angle to servo limits
        this.targetRoll = Math.max(Math.min(roll, 1), 0);
    }

    public double getTargetRoll() {
        return this.targetRoll;
    }

    public double getTargetPitch() {
        return this.targetPitch;
    }

    public double getCurrentPitch() {
        return encoder.getPosition();
    }

    public double getCurrentRoll() {
        return (m_rollServo.get() - 0.5) * 90.0;  // Convert servo position back to degrees
    }

    public void update() {
        // Convert target angles to compensated angles
        double compensatedPitch = targetPitch + calculatePitchCompensation(targetRoll);
        // double compensatedRoll = targetRoll + calculateRollCompensation(targetPitch);
        double compensatedRoll = targetRoll;

        // Update servo and motor with compensated values
        // m_rollServo.set(mapRollToServoPosition(compensatedRoll));
        m_rollServo.set(compensatedRoll);
        pitchPid.setReference(compensatedPitch, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void stop() {
        m_pitchMotor.stopMotor();
    }

    private double calculatePitchCompensation(double rollAngle) {
        // Ignore very small angles to prevent tiny oscillations
        if (Math.abs(rollAngle) < MIN_COMPENSATION_ANGLE) return 0.0;
        // For every degree of roll, pitch changes by 1/3 degree
        // return rollAngle * wristDiffRatio;
        return rollAngle * wristDiffRatio - (targetPitch * (1 - wristDiffRatio)) + 1 - wristDiffRatio;
    }
    
    private double calculateRollCompensation(double pitchAngle) {
        // Ignore very small angles to prevent tiny oscillations
        if (Math.abs(pitchAngle) < MIN_COMPENSATION_ANGLE) return 0.0;
        // For every degree of pitch, roll changes by 1/3 degree
        return pitchAngle * wristDiffRatio;
    }

    // /* Sets wrist pitch and roll positions from 0.0 to 1.0. */
    // public void SetWristPosition(double pitch, double roll) {
    //   pitch = Math.max(Math.min(pitch, 1), 0);
    //   roll = Math.max(Math.min(roll, 1), 0);
    //   m_pitchMotor.set(roll * wristDiffRatio - (pitch * (1 - wristDiffRatio)) + 1 - wristDiffRatio);
    //   m_rollServo.set(roll);
    // //   currentPitch = pitch;
    // //   currentRoll = roll;
    // }

    private double mapRollToServoPosition(double rollAngle) {
        double minServo = 0.0;
        double maxServo = 1.0;
        double minAngle = -45.0;
        double maxAngle = 45.0;

        return (rollAngle - minAngle) / (maxAngle - minAngle) * (maxServo - minServo) + minServo;
    }

    @Override
    public void periodic() {
        // Display encoder position and velocity in degrees
        SmartDashboard.putNumber("Pitch Angle (deg)", encoder.getPosition());
        SmartDashboard.putNumber("Pitch Velocity (deg/s)", encoder.getVelocity());
        SmartDashboard.putNumber("Pitch Output (%)", m_pitchMotor.getAppliedOutput());
        SmartDashboard.putNumber("Pitch Output (amps)", m_pitchMotor.getOutputCurrent());
        SmartDashboard.putNumber("Target Pitch (deg)", targetPitch);
        SmartDashboard.putNumber("Position Error (deg)", targetPitch - encoder.getPosition());

        // Display compensation calculations
        SmartDashboard.putNumber("Compensated Pitch", targetPitch + calculatePitchCompensation(targetRoll));
        SmartDashboard.putNumber("Compensated Roll", targetRoll + calculateRollCompensation(targetPitch));
        SmartDashboard.putNumber("Roll Servo Position", m_rollServo.get());

        // Handle encoder reset request
        if (SmartDashboard.getBoolean("Reset Encoder", false)) {
            SmartDashboard.putBoolean("Reset Encoder", false);
            encoder.setPosition(0);
        }
    }
}
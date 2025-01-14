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
import edu.wpi.first.math.controller.*;
import frc.robot.Constants.ArmConstants;

public class WristSubsystem extends SubsystemBase {
    private final Servo m_rollServo; // Rev Smart Servo for roll
    private final SparkMax m_pitchMotor; // Neo 550 for pitch

    private SparkMaxConfig pitchMotorConfig; // Config for Neo 550
    private SparkClosedLoopController pitchPid; // PID controller for pitch
    private RelativeEncoder encoder; // Encoder for Neo 550

    // Arm Feed Forward, est values (kG = 0.19, kV = 1.32, kA = 0.01) rads (kG = 0.19, kV = 0.02, kA = 0.00) deg
    private ArmFeedforward arbFF;

    private double targetRoll = 0; // Desired roll angle in degrees
    private double targetPitch = 0; // Desired pitch angle in degrees
    
    // Constants for pitch angle conversion
    private static final double GEAR_RATIO = 135.0; // 45:1 Reduction gearbox, 3:1 (135:1) chain?
    private static final double ENCODER_COUNTS_PER_MOTOR_REV = 42.0;
    private static final double DEGREES_PER_REV = 360.0;
    private static final double DEGREES_PARALLEL_TO_GROUND = 3.0;
    
    // Differential ratio between roll and pitch
    private final double wristDiffRatio = 1.0/3.0;
    
    // Minimum angle for compensation calculations
    private static final double MIN_COMPENSATION_ANGLE = 0.1;
    
    public WristSubsystem() {
        m_rollServo = new Servo(ArmConstants.kWristDiffServoPort);
        m_pitchMotor = new SparkMax(ArmConstants.kWristPitchMotorPort, MotorType.kBrushless);
        pitchPid = m_pitchMotor.getClosedLoopController();
        encoder = m_pitchMotor.getEncoder();
        pitchMotorConfig = new SparkMaxConfig();

        arbFF = new ArmFeedforward(0.19, 0.02, 0.0);

        // Set brake mode
        pitchMotorConfig
            .inverted(false)
            .smartCurrentLimit(10) // Trying to minimize damage to wrist from powerful motor
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
            .p(0.3)
            .i(0.0)
            .d(0.0)
            .outputRange(-1, 0.1);

        // Set soft limits in degrees
        pitchMotorConfig.softLimit
            .forwardSoftLimit(DEGREES_PARALLEL_TO_GROUND)  // Maximum 7 degrees down
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(0) // Maximum 0 degrees up (relative encoder, should be 0?)
            .reverseSoftLimitEnabled(true);

        m_pitchMotor.configure(pitchMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setTargetAngles(double roll, double pitch) {
        // Clamp pitch angle to soft limits
        this.targetPitch = Math.max(Math.min(pitch, DEGREES_PARALLEL_TO_GROUND), 0.0);
        // Clamp roll angle to servo limits
        this.targetRoll = Math.max(Math.min(roll, 0.5), 0); // Full 1 is too far
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
        return m_rollServo.get();
    }

    public void update() {
        // Update servo and motor with compensated values
        m_rollServo.set(targetRoll);
        pitchPid.setReference(
            targetPitch,
            ControlType.kPosition, 
            ClosedLoopSlot.kSlot0
        );
    }

    public void stop() {
        m_pitchMotor.stopMotor();
    }

    private double calculatePitchCompensation(double rollAngle) {
        // Ignore very small angles to prevent tiny oscillations
        if (Math.abs(rollAngle) < MIN_COMPENSATION_ANGLE) return 0.0;
        // For every degree of roll, pitch changes by 1/3 degree
        return rollAngle * wristDiffRatio;
        // return rollAngle * wristDiffRatio - (targetPitch * (1 - wristDiffRatio)) + 1 - wristDiffRatio;
    }

    @Override
    public void periodic() {
        // Display encoder position and velocity in degrees
        SmartDashboard.putNumber("Pitch Angle (deg)", encoder.getPosition());
        SmartDashboard.putNumber("Target Pitch (deg)", targetPitch);
        SmartDashboard.putNumber("Pitch Velocity (deg/s)", encoder.getVelocity());
        SmartDashboard.putNumber("Pitch Output (%)", m_pitchMotor.getAppliedOutput());
        SmartDashboard.putNumber("Pitch Output (amps)", m_pitchMotor.getOutputCurrent());
        //SmartDashboard.putNumber("Target Roll", targetRoll);
        SmartDashboard.putNumber("Position Error (deg)", targetPitch - encoder.getPosition());

        // Display compensation calculations
        SmartDashboard.putNumber("Compensated Pitch", targetPitch + calculatePitchCompensation(targetRoll));
        SmartDashboard.putNumber("Roll Servo Position", m_rollServo.get());

        // Handle encoder reset request
        if (SmartDashboard.getBoolean("Reset Encoder", false)) {
            SmartDashboard.putBoolean("Reset Encoder", false);
            encoder.setPosition(0);
        }
    }
}
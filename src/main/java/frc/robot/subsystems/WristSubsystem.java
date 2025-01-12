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
    private SparkClosedLoopController  pitchPid; // PID controller for pitch
    private RelativeEncoder encoder; // Encoder for Neo 550
    private SparkLimitSwitch forwardLimitSwitch; // SoftStop for Neo 550
    private SparkLimitSwitch reverseLimitSwitch;

    private double targetRoll = 0; // Desired roll angle
    private double targetPitch = 0; // Desired pitch angle
    private final double wristDiffRatio = 1.0/3.0; // WristDiff revolutions per wirstPitch revolution

    public WristSubsystem() {
        m_rollServo = new Servo(ArmConstants.kWristDiffServoPort);
        m_pitchMotor = new SparkMax(ArmConstants.kWristPitchMotorPort, MotorType.kBrushless);
        pitchPid  = m_pitchMotor.getClosedLoopController();
        encoder = m_pitchMotor.getEncoder();
        pitchMotorConfig = new SparkMaxConfig();

        // Set brake mode
        pitchMotorConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        // Set up encoders
        pitchMotorConfig.encoder
            .positionConversionFactor(1) // Need to measure.
            .velocityConversionFactor(1);
        // Configure Pid loop
        pitchMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.1, 0.0, 0.0);
        // Enable limit switches to stop the motor when they are closed
        pitchMotorConfig.limitSwitch
            .forwardLimitSwitchType(Type.kNormallyOpen)
            .forwardLimitSwitchEnabled(true)
            .reverseLimitSwitchType(Type.kNormallyOpen)
            .reverseLimitSwitchEnabled(true);
        // Set the soft limits to stop the motor at -50 and 50 rotations
        pitchMotorConfig.softLimit
            .forwardSoftLimit(50)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(-50)
            .reverseSoftLimitEnabled(true);

        m_pitchMotor.configure(pitchMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setTargetAngles(double roll, double pitch) {
        this.targetRoll = roll;
        this.targetPitch = pitch;
    }

    public double getTargetRoll() {
        return this.targetRoll;
    }

    public double getTargetPitch() {
        return this.targetPitch;
    }

    public void update(double currentPitch) {
        m_rollServo.set(mapRollToServoPosition(targetRoll));
        pitchPid.setReference(targetPitch, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void stop() {
        m_pitchMotor.stopMotor();
    }

    private double mapRollToServoPosition(double rollAngle) {
        double minServo = 0.0;
        double maxServo = 1.0;
        double minAngle = -90.0;
        double maxAngle = 90.0;

        return (rollAngle - minAngle) / (maxAngle - minAngle) * (maxServo - minServo) + minServo;
        //  m_wristPitch.set(roll * wristDiffRatio - (pitch * (1 - wristDiffRatio)) + 1 - wristDiffRatio);
    }

    @Override
    public void periodic() {
        // Display encoder position and velocity
        SmartDashboard.putNumber("Actual Position", encoder.getPosition());
        SmartDashboard.putNumber("Actual Velocity", encoder.getVelocity());

        SmartDashboard.putBoolean("Forward Limit Reached", forwardLimitSwitch.isPressed());
        SmartDashboard.putBoolean("Reverse Limit Reached", reverseLimitSwitch.isPressed());
        SmartDashboard.putNumber("Applied Output", m_pitchMotor.getAppliedOutput());

        if (SmartDashboard.getBoolean("Reset Encoder", false)) {
            SmartDashboard.putBoolean("Reset Encoder", false);
            // Reset the encoder position to 0
            encoder.setPosition(0);
        }
    }
}

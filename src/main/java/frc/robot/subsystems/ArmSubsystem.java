// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPositions;

import com.revrobotics.spark.SparkMax;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  private final WPI_TalonSRX m_armRotate = new WPI_TalonSRX(ArmConstants.kArmRotateMotorPort);
  private final WPI_TalonSRX m_armExtend = new WPI_TalonSRX(ArmConstants.kArmExtendMotorPort);

  // Temporarily switching to Bag Motor
  private final SparkMax m_intake = new SparkMax(ArmConstants.kArmIntakeMotorPort, MotorType.kBrushed);
  // private final SparkMax m_intake = new SparkMax(ArmConstants.kArmIntakeMotorPort, MotorType.kBrushless);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {

    m_armRotate.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 
                                            ArmConstants.kPIDLoopIdx,
				                            Constants.kTimeoutMs);
    m_armExtend.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 
                                    ArmConstants.kPIDLoopIdx,
                            Constants.kTimeoutMs);

      // Set Brake Mode
    m_armExtend.setNeutralMode(NeutralMode.Brake);
    m_armRotate.setNeutralMode(NeutralMode.Brake);

    // Enable Extention PID Stuff
		m_armExtend.config_kP(ArmConstants.kPIDLoopIdx, ArmConstants.kExtendP, Constants.kTimeoutMs);
		m_armExtend.config_kI(ArmConstants.kPIDLoopIdx, ArmConstants.kExtendI, Constants.kTimeoutMs);
    m_armExtend.config_kD(ArmConstants.kPIDLoopIdx, ArmConstants.kExtendD, Constants.kTimeoutMs);
    
    // Acceleration of 4 inches / s^2
    m_armExtend.configMotionAcceleration(positionInchesToTicks(4) * 0.1);
    // Peak velocity of 2 inches per second
    m_armExtend.configMotionCruiseVelocity(positionInchesToTicks(2) * 0.1);
    // Use trapezoidal curve
    m_armExtend.configMotionSCurveStrength(0);

    // Enable Rotate PID Stuff
		m_armRotate.config_kP(ArmConstants.kPIDLoopIdx, ArmConstants.kRotateP, Constants.kTimeoutMs);
		m_armRotate.config_kI(ArmConstants.kPIDLoopIdx, ArmConstants.kRotateI, Constants.kTimeoutMs);
    m_armRotate.config_kD(ArmConstants.kPIDLoopIdx, ArmConstants.kRotateD, Constants.kTimeoutMs);

    // Acceleration of 180 degrees / s^2
    m_armRotate.configMotionAcceleration(angleDegreesToTicks(180) * 0.1);
    // Peak velocity of 90 degrees per second
    m_armRotate.configMotionCruiseVelocity(angleDegreesToTicks(90) * 0.1);
    // Use trapezoidal curve
    m_armRotate.configMotionSCurveStrength(0);

    zeroArmAngle();
    //zeroArmLength();
  }

  public void cancelPid() {
    m_armExtend.stopMotor();
    m_armRotate.stopMotor();
    // Does this work?
  }

  /**
   * Returns the total current draw of the arm subsystem.
   *
   * @return The arm motors' total current draw (in amps).
   */
  public double getCurrentDraw() {
    return m_armRotate.getStatorCurrent() +
        m_armExtend.getStatorCurrent() +
        m_intake.getOutputCurrent();
  }

  public void SetExtensionPos(double setPos, boolean doSmoothing){
    // Ticks are negative...
    if (setPos < ArmConstants.kExtendMaxTicks) {
      setPos = ArmConstants.kExtendMaxTicks;
    }
    m_armExtend.set(doSmoothing ? TalonSRXControlMode.MotionMagic : TalonSRXControlMode.Position, setPos);
  }

  public void SetRotationPos(double setPos, boolean doSmoothing) {
    m_armRotate.set(
      doSmoothing ? TalonSRXControlMode.MotionMagic : TalonSRXControlMode.Position,
      setPos,
      DemandType.ArbitraryFeedForward,
      ArmConstants.kRotateGravityScalar * Math.cos(currentArmAngle() * Math.PI / 180)
      );
  }

  public void SetIntakeSpeed(double speed) {
    m_intake.set(speed);
  }
  
  // Arm angle functions

  public static double angleTicksToDegrees(double ticks) {
    return ticks / ArmConstants.kAngleSensorUnitsPerRotation * 360;
  }

  public static double angleDegreesToTicks(double degrees) {
    return degrees / 360 * ArmConstants.kAngleSensorUnitsPerRotation;
  }

  public double currentArmAngle() {
    return angleTicksToDegrees(m_armRotate.getSelectedSensorPosition()) + ArmPositions.kArmRestingDegrees;
  }

  public double targetArmAngle() {
    return angleTicksToDegrees(m_armRotate.getClosedLoopTarget()) + ArmPositions.kArmRestingDegrees;
  }

  // Arm length functions

  public static double positionTicksToInches(double ticks) {
    double percentTicks = ticks / ArmConstants.kExtendMaxTicks;
    // ticksPerInch is a function of position
    double curTicksPerInch = percentTicks * (ArmConstants.kExtendedTicksPerInch - ArmConstants.kRetractedTicksPerInch) + ArmConstants.kRetractedTicksPerInch;

    double startInchesPerTick = 1.0 / ArmConstants.kRetractedTicksPerInch;
    //inchesPerTick decreases over distance
    double curInchesPerTick = 1.0 / curTicksPerInch;
    
    return ticks * (curInchesPerTick + startInchesPerTick / 2);
  }

  // Note: positive inches returns a negative tick offset
  public static double positionInchesToTicks(double inches) {
    double percentInches = inches / ArmConstants.kExtendMaxInches;
    // inchesPerTick is a function of position
    double curInchesPerTick = percentInches * (1.0/ArmConstants.kExtendedTicksPerInch - 1.0/ArmConstants.kRetractedTicksPerInch) + 1.0/ArmConstants.kRetractedTicksPerInch;

    double startTicksPerInch = ArmConstants.kRetractedTicksPerInch;
    //ticksPerInch increases over distance
    double curTicksPerInch = 1.0 / curInchesPerTick;
    
    return inches * (startTicksPerInch + curTicksPerInch / 2);
  }

  public void zeroArmLength() {
    m_armExtend.setSelectedSensorPosition(0);
    SetExtensionPos(0, false);
  }

  public void zeroArmAngle() {
    m_armRotate.setSelectedSensorPosition(0);
    SetRotationPos(0, false);
  }

  private final double sampleMillis = 500;
  // private final double zeroingAngleStepTicks = angleDegreesToTicks(2);
  private final double zeroingLengthStepTicks = positionInchesToTicks(0.5);
  
  private boolean isZeroing = false;
  private boolean zeroingLengthDone = false;
  // private boolean zeroingAngleDone = false;
  private double zeroingLengthStepPos = 0;
  // private double zeroingAngleStepPos = 0;
  private double zeroingStepTime = System.currentTimeMillis();
  
  public void autoZeroArm() {
    if (!isZeroing) {
      // Update step time and positions
      zeroingStepTime = System.currentTimeMillis();
      // zeroingAngleStepPos = m_armRotate.getSelectedSensorPosition();
      zeroingLengthStepPos = m_armExtend.getSelectedSensorPosition();
      isZeroing = true;
    }

    if (!autoZeroDone()) {
      // Slowly move arm for a short amount of time
      if (zeroingStepTime + sampleMillis > System.currentTimeMillis()) {
        // m_armRotate.set(TalonSRXControlMode.PercentOutput, zeroingAngleDone ? 0 : -0.1);
        m_armExtend.set(TalonSRXControlMode.PercentOutput, zeroingLengthDone ? 0 : 0.3);
      } else {
        // If angle has changed enough, keep going; otherwise stop and zero
        // if (!zeroingAngleDone && zeroingAngleStepPos - zeroingAngleStepTicks > m_armRotate.getSelectedSensorPosition()) {
        //   zeroingAngleStepPos = m_armRotate.getSelectedSensorPosition();
        // } else {
        //   m_armRotate.set(TalonSRXControlMode.PercentOutput, 0);
        //   zeroArmAngle();
        //   zeroingAngleDone = true;
        // }
        zeroArmAngle();

        // If length has changed enough, keep going; otherwise stop and zero
        if (!zeroingLengthDone && zeroingLengthStepPos - zeroingLengthStepTicks < m_armExtend.getSelectedSensorPosition()) {
          zeroingLengthStepPos = m_armExtend.getSelectedSensorPosition();
        } else {
          m_armExtend.set(TalonSRXControlMode.PercentOutput, 0);
          zeroArmLength();
          zeroingLengthDone = true;
        }

        // Update step time
        zeroingStepTime = System.currentTimeMillis();
      }
    }
  }

  public boolean autoZeroDone() {
    return zeroingLengthDone;// && zeroingAngleDone;
  }

  public void resetZeroingFlag() {
    m_armRotate.set(TalonSRXControlMode.PercentOutput, 0);
    m_armExtend.set(TalonSRXControlMode.PercentOutput, 0);
    zeroingLengthDone = false;
    // zeroingAngleDone = false;
    isZeroing = false;
  }

  public double currentArmAngleTicks() {
    return m_armRotate.getSelectedSensorPosition();
  }

  public double targetArmAngleTicks() {
    return m_armRotate.getClosedLoopTarget();
  }

  public double targetArmLengthTicks() {
    return m_armExtend.getClosedLoopTarget();
  }


  public double currentArmExtendTicks() {
    return m_armExtend.getSelectedSensorPosition();
  }

  public double currentArmInches() {
    return positionTicksToInches(m_armExtend.getSelectedSensorPosition());
  }

  public double targetArmInches() {
    return positionTicksToInches(m_armExtend.getClosedLoopTarget());
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    SmartDashboard.putNumber("Arm Rotate Encoder Ticks", m_armRotate.getSelectedSensorPosition());
    SmartDashboard.putNumber("Arm Rotate Setpoint", m_armRotate.getClosedLoopTarget());
    SmartDashboard.putNumber("Arm Rotate Degrees", currentArmAngle());
    SmartDashboard.putNumber("Arm Rotate PID Error", m_armRotate.getClosedLoopError());
    SmartDashboard.putNumber("Arm Rotate Motor Amps", m_armRotate.getStatorCurrent());

    SmartDashboard.putNumber("Arm Extend Encoder Ticks", m_armExtend.getSelectedSensorPosition());
    SmartDashboard.putNumber("Arm Extend Setpoint", m_armExtend.getClosedLoopTarget());
    SmartDashboard.putNumber("Arm Extend Inches", currentArmInches());
    SmartDashboard.putNumber("Arm Extend PID Error", m_armExtend.getClosedLoopError());
    SmartDashboard.putNumber("Arm Extend Motor Amps", m_armExtend.getStatorCurrent());

    SmartDashboard.putBoolean("Zeroing Done", autoZeroDone());
    SmartDashboard.putNumber("Intake Current", m_intake.getOutputCurrent());
  }
}
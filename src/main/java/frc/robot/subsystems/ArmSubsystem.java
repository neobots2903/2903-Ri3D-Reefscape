// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  private final WPI_TalonSRX m_armRotate = new WPI_TalonSRX(ArmConstants.kArmRotateMotorPort);
  private final WPI_TalonSRX m_armExtend = new WPI_TalonSRX(ArmConstants.kArmExtendMotorPort);
  private final SparkMax m_armIntake = new SparkMax(ArmConstants.kArmIntakeMotorPort, MotorType.kBrushless);

  // // Only rotates the wrist's pitch
  // private final Servo m_wristPitch = new Servo(ArmConstants.kWristPitchServoPort);
  // // Rotates the wrist's pitch and roll equally
  // private final Servo m_wristDiff = new Servo(ArmConstants.kWristDiffServoPort);

  // WristDiff revolutions per wirstPitch revolution
  private final double wristDiffRatio = 1.0/3.0;

  // Measured from 0.0 to 1.0 (percent of servo's range)
  private double currentPitch = 0.0;
  private double currentRoll = 0.0;

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
    m_armRotate.setNeutralMode(NeutralMode.Brake); // NEEDS TO BE BRAKE

    // Enable Extention PID Stuff
		m_armExtend.config_kP(ArmConstants.kPIDLoopIdx, ArmConstants.kExtendP, Constants.kTimeoutMs);
		m_armExtend.config_kI(ArmConstants.kPIDLoopIdx, ArmConstants.kExtendI, Constants.kTimeoutMs);
    m_armExtend.config_kD(ArmConstants.kPIDLoopIdx, ArmConstants.kExtendD, Constants.kTimeoutMs);
    m_armExtend.config_kF(ArmConstants.kPIDLoopIdx, ArmConstants.kExtendF, Constants.kTimeoutMs);

    // Enable Rotate PID Stuff
		m_armRotate.config_kP(ArmConstants.kPIDLoopIdx, ArmConstants.kRotateP, Constants.kTimeoutMs);
		m_armRotate.config_kI(ArmConstants.kPIDLoopIdx, ArmConstants.kRotateI, Constants.kTimeoutMs);
    m_armRotate.config_kD(ArmConstants.kPIDLoopIdx, ArmConstants.kRotateD, Constants.kTimeoutMs);

    zeroArmAngle();
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
        m_armIntake.getOutputCurrent();
  }

  public double GetWristPitch() {
    return currentPitch;
  }

  public double GetWristRoll() {
    return currentRoll;
  }

  // public void SetWristPitch(double pitch) {
  //   SetWristPosition(pitch, currentRoll);
  // }

  // public void SetWristRoll(double roll) {
  //   SetWristPosition(currentPitch, roll);
  // }

  // /* Sets wrist pitch and roll positions from 0.0 to 1.0. */
  // public void SetWristPosition(double pitch, double roll) {
  //   pitch = Math.max(Math.min(pitch, 1), 0);
  //   roll = Math.max(Math.min(roll, 1), 0);
  //   m_wristPitch.set(roll * wristDiffRatio - (pitch * (1 - wristDiffRatio)) + 1 - wristDiffRatio);
  //   m_wristDiff.set(roll);
  //   currentPitch = pitch;
  //   currentRoll = roll;
  // }

  public void SetExtensionPos(double setPos){
    m_armExtend.set(TalonSRXControlMode.Position, setPos);
  }

  public void SetRotationPos(double setPos) {
    m_armRotate.set(
      TalonSRXControlMode.Position,
      setPos,
      DemandType.ArbitraryFeedForward,
      ArmConstants.kRotateGravityScalar * Math.cos(currentArmAngle() * Math.PI / 180)
      );
  }

  public void SetIntakeSpeed(double speed) {
    m_armIntake.set(speed);
  }

  public static double angleTicksToDegrees(double ticks) {
    return ticks / ArmConstants.kSensorUnitsPerRotation * 360;
  }

  public static double angleDegreesToTicks(double degrees) {
    return degrees / 360 * ArmConstants.kSensorUnitsPerRotation;
  }

  public void zeroArmAngle() {
    m_armRotate.setSelectedSensorPosition(0);
  }

  public double currentArmAngle() {
    return angleTicksToDegrees(m_armRotate.getSelectedSensorPosition()) + ArmPositions.kArmRestingDegrees;
  }

  public double targetArmAngle() {
    return angleTicksToDegrees(m_armRotate.getClosedLoopTarget()) + ArmPositions.kArmRestingDegrees;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    SmartDashboard.putNumber("Arm Extension Encoder Ticks", m_armExtend.getSelectedSensorPosition());
    SmartDashboard.putNumber("Arm Rotate Encoder Ticks", m_armRotate.getSelectedSensorPosition());
    SmartDashboard.putNumber("Arm Rotate Setpoint", m_armRotate.getClosedLoopTarget());
    SmartDashboard.putNumber("Arm Rotate Degrees", currentArmAngle());
    SmartDashboard.putNumber("PID Error", m_armRotate.getClosedLoopError());
    SmartDashboard.putNumber("Motor Amps", m_armRotate.getStatorCurrent());
  }
}
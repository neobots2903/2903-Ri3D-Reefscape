// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.ArmConstants;

import com.ctre.phoenix.motorcontrol.can.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  private final WPI_TalonSRX m_armRotate = new WPI_TalonSRX(ArmConstants.kArmRotateMotorPort);
  private final WPI_TalonSRX m_armExtend = new WPI_TalonSRX(ArmConstants.kArmExtendMotorPort);
  private final SparkMax m_armIntake = new SparkMax(ArmConstants.kArmIntakeMotorPort, MotorType.kBrushless);

  // Only rotates the wrist's pitch
  private final Servo m_wristPitch = new Servo(ArmConstants.kWristPitchServoPort);
  // Rotates the wrist's pitch and roll equally
  private final Servo m_wristDiff = new Servo(ArmConstants.kWristDiffServoPort);

  // WristDiff revolutions per wirstPitch revolution
  private final double wristDiffRatio = 1.0/3.0;

  // Measured from 0.0 to 1.0 (percent of servo's range)
  private double currentPitch = 0.0;
  private double currentRoll = 0.0;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
  }

  public double GetWristPitch() {
    return currentPitch;
  }

  public double GetWristRoll() {
    return currentRoll;
  }

  public void SetWristPitch(double pitch) {
    SetWristPosition(pitch, currentRoll);
  }

  public void SetWristRoll(double roll) {
    SetWristPosition(currentPitch, roll);
  }

  /* Sets wrist pitch and roll positions from 0.0 to 1.0. */
  public void SetWristPosition(double pitch, double roll) {
    m_wristPitch.set(roll * wristDiffRatio - pitch);
    m_wristDiff.set(roll);
    currentPitch = pitch;
    currentRoll = roll;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
  }
}
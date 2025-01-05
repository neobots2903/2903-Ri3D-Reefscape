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
  private final Servo m_wristPitch = new Servo(ArmConstants.kWristPitchServoPort);
  private final Servo m_wristDiff = new Servo(ArmConstants.kWristDiffServoPort);
  private final SparkMax m_armIntake = new SparkMax(ArmConstants.kArmIntakeMotorPort, MotorType.kBrushless);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
  }
}
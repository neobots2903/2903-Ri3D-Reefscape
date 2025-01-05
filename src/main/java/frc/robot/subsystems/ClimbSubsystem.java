// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ClimbConstants;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

    // Note: limit switches and encoders are currently undecided for this class
  private final WPI_VictorSPX m_climbLeft = new WPI_VictorSPX(ClimbConstants.kClimbLeftMotorPort);
  private final WPI_VictorSPX m_climbRight = new WPI_VictorSPX(ClimbConstants.kClimbRightMotorPort);
  private final WPI_TalonSRX m_climbEngage = new WPI_TalonSRX(ClimbConstants.kClimbEngageMotorPort);

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    m_climbLeft.setInverted(true);
    m_climbRight.setInverted(false);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;

// TODO:
/*
 * Need to find out motor controller type (Spark Max, Talon, Victor SP, Victor SPX).
 * Find out which encoders the build team will grace us with, if any...
 */

 // Hardware:
 /*
  * 2 CIM for climb.
  * 1 mini CIM for engage.
  */

// NOTES:
/*
 * Could also do position control, but I was thinking that Current would be easier to
 * work with and less likely to kill motors. Empirically find amps under normal climb, then
 * use as target current.
 */

public class ClimbSubsystem extends SubsystemBase {

  private final WPI_TalonSRX m_frontClimb = new WPI_TalonSRX(ClimbConstants.kFrontClimbMotorPort);
  private final WPI_TalonSRX m_rearClimb = new WPI_TalonSRX(ClimbConstants.kRearClimbMotorPort);
  private final WPI_TalonSRX m_engageClimb = new WPI_TalonSRX(ClimbConstants.kEngageClimbMotorPort);


  public ClimbSubsystem() {
    // Set one climb to follower
    m_rearClimb.set(TalonSRXControlMode.Follower, ClimbConstants.kFrontClimbMotorPort);

    // Set Brake Mode
    m_rearClimb.setNeutralMode(NeutralMode.Brake);
    m_frontClimb.setNeutralMode(NeutralMode.Brake);

    // Enable PID Stuff
		m_engageClimb.config_kP(ClimbConstants.kPIDLoopIdx, ClimbConstants.kP, ClimbConstants.kTimeoutMs);
		m_engageClimb.config_kI(ClimbConstants.kPIDLoopIdx, ClimbConstants.kI, ClimbConstants.kTimeoutMs);
    m_engageClimb.config_kD(ClimbConstants.kPIDLoopIdx, ClimbConstants.kD, ClimbConstants.kTimeoutMs);
  }

  public void setClimbPower(double percentOutput){
    m_frontClimb.set(TalonSRXControlMode.PercentOutput, percentOutput);
  }

  // Will need a PID loop in current mode(?) for keeping constant pressure on the cage.
  public void setEngagePower(int ampsOutput){
    m_engageClimb.set(TalonSRXControlMode.Current, ampsOutput); // 40 Amp breaker in PDP
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Engage Current Voltage", m_engageClimb.getStatorCurrent());
    SmartDashboard.putNumber("Engage PID Target", m_engageClimb.getClosedLoopTarget());
    SmartDashboard.putNumber("Engage PID Error", m_engageClimb.getClosedLoopError(ClimbConstants.kPIDLoopIdx));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

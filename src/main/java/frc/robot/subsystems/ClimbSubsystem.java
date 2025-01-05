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

  private final WPI_TalonSRX m_RightClimb = new WPI_TalonSRX(ClimbConstants.kRightClimbMotorPort);
  private final WPI_TalonSRX m_LeftClimb = new WPI_TalonSRX(ClimbConstants.kLeftClimbMotorPort);
  private final WPI_TalonSRX m_engageClimb = new WPI_TalonSRX(ClimbConstants.kEngageClimbMotorPort);

  private boolean isEngaged;

  public ClimbSubsystem() {
    // Set one climb to follower
    m_RightClimb.set(TalonSRXControlMode.Follower, ClimbConstants.kRightClimbMotorPort);

    // Set Brake Mode
    m_RightClimb.setNeutralMode(NeutralMode.Brake);
    m_LeftClimb.setNeutralMode(NeutralMode.Brake);

    // Enable PID Stuff
		m_engageClimb.config_kP(ClimbConstants.kPIDLoopIdx, ClimbConstants.kP, ClimbConstants.kTimeoutMs);
		m_engageClimb.config_kI(ClimbConstants.kPIDLoopIdx, ClimbConstants.kI, ClimbConstants.kTimeoutMs);
    m_engageClimb.config_kD(ClimbConstants.kPIDLoopIdx, ClimbConstants.kD, ClimbConstants.kTimeoutMs);

    // Set class attributes
    isEngaged = false; 
  }

  public void setClimbPower(double percentOutput){
    m_LeftClimb.set(TalonSRXControlMode.PercentOutput, percentOutput);
  }

  // Will need a PID loop in current mode(?) for keeping constant pressure on the cage.
  public void setEngagePower(int ampsOutput){
    m_engageClimb.set(TalonSRXControlMode.Current, ampsOutput); // 40 Amp breaker in PDP
    this.setEngagedStatus(); //any time this is run re-set the engaged status 
  }

  public void setEngagedStatus(){//false is disabled, true is enabled
    isEngaged = !isEngaged;
  }

  public boolean getEngagedStatus(){
    return isEngaged;
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

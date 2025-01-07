// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  private final VictorSPX m_engageClimb = new VictorSPX(ClimbConstants.kEngageClimbMotorPort);
  private final VictorSP m_LeftClimb = new VictorSP(ClimbConstants.kLeftClimbMotorPort);

  private boolean isEngaged;

  public ClimbSubsystem() {

    // Set Brake Mode
    m_RightClimb.setNeutralMode(NeutralMode.Brake);

    m_LeftClimb.setInverted(true);

    // Set class attributes
    isEngaged = false; 
  }

  public void setClimbPower(double percentOutput){
    m_LeftClimb.set(percentOutput);
    m_RightClimb.set(TalonSRXControlMode.PercentOutput, percentOutput);
  }

  public void setEngagePower(double percentOutput){
    m_engageClimb.set(VictorSPXControlMode.PercentOutput, percentOutput); // 40 Amp breaker in PDP
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

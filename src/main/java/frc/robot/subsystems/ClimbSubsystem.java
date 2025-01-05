// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
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

    // Configure the engage Talon's selected sensor as local QuadEncoder
    m_engageClimb.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder,	// Local Feedback Source
                                                0,		            // PID Slot for Source [0, 1]
                                                Constants.kTimeoutMs);		// Configuration Timeout
  }

  public void setClimbPower(double percentOutput){
    m_frontClimb.set(TalonSRXControlMode.PercentOutput, percentOutput);
  }

  // Will need a PID loop in current mode(?) for keeping constant pressure on the cage.
  public void setEngagePower(double percentOutput){
    m_engageClimb.set(TalonSRXControlMode.PercentOutput, percentOutput);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.LEDConstants;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  
  // Since data timings can be as fast as 0.7 usecs, a separate co-processor must drive the LEDs.
  // This class will send RGB values to the co-processor at a more leisurely pace.
  private final DigitalOutput m_ledData = new DigitalOutput(LEDConstants.kLedDataDigitalPort);

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
  }


}
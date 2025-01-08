// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.LEDConstants;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  
  private final SerialPort m_ledSerial = new SerialPort(9600, Port.kUSB);

  public enum LEDMode {
    Idle,
    Blue,
    Red,
    Demo,
    Off
  }

  LEDMode currentMode;

  public void setLEDMode(LEDMode mode) {
    currentMode = mode;
    m_ledSerial.writeString(Integer.toString(mode.ordinal()));
    m_ledSerial.flush();
  }

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    setLEDMode(LEDMode.Idle);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
  }


}
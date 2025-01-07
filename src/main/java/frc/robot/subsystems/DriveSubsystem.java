// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  
  private final WPI_TalonSRX m_frontLeft = new WPI_TalonSRX(DriveConstants.kFrontLeftMotorPort);
  private final WPI_TalonSRX m_rearLeft = new WPI_TalonSRX(DriveConstants.kRearLeftMotorPort);
  private final WPI_TalonSRX m_frontRight = new WPI_TalonSRX(DriveConstants.kFrontRightMotorPort);
  private final WPI_TalonSRX m_rearRight = new WPI_TalonSRX(DriveConstants.kRearRightMotorPort);

  private final MecanumDrive m_drive = new MecanumDrive(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight);

  // The gyro sensor
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  // The accelerometer sensor
  BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

  // Accelerometer variables
  private final double GFORCE_TO_METERS_PER_SECOND_SQUARED = 9.80665;
  private double lastAccelX, lastAccelY;
  private double accelDistanceX, accelDistanceY;
  private double accelVelocityX, accelVelocityY;
  private long lastAccelMeasureTime;



  // Odometry class for tracking robot pose
  MecanumDriveOdometry m_odometry =
      new MecanumDriveOdometry(
          DriveConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          new MecanumDriveWheelPositions());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    SendableRegistry.addChild(m_drive, m_frontLeft);
    SendableRegistry.addChild(m_drive, m_rearLeft);
    SendableRegistry.addChild(m_drive, m_frontRight);
    SendableRegistry.addChild(m_drive, m_rearRight);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_frontRight.setInverted(true);
    m_rearRight.setInverted(true);
    m_rearLeft.setInverted(true); // Wires backwards.
    // Initialize accel-based location estimation
    resetAccelDistance();
    SmartDashboard.putBoolean("Reset Accel Data", false);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        new MecanumDriveWheelPositions(
            m_frontLeft.getSelectedSensorPosition(),
            m_rearLeft.getSelectedSensorPosition(),
            m_frontRight.getSelectedSensorPosition(),
            m_rearRight.getSelectedSensorPosition()));

    SmartDashboard.putNumber("Front Left Encoder", m_frontLeft.getSelectedSensorPosition());
    SmartDashboard.putNumber("Rear Left Encoder", m_rearLeft.getSelectedSensorPosition());
    SmartDashboard.putNumber("Front Right Encoder", m_frontRight.getSelectedSensorPosition());
    SmartDashboard.putNumber("Rear Right Encoder", m_rearRight.getSelectedSensorPosition());

    SmartDashboard.putNumber("Gyro", m_gyro.getAngle());

    SmartDashboard.putNumber("X", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Y", m_odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Rotation", m_odometry.getPoseMeters().getRotation().getDegrees());
    
    // Acceleration-based odometry
    if (SmartDashboard.getBoolean("Reset Accel Data", false)) {
      SmartDashboard.putBoolean("Reset Accel Data", false);
      resetAccelDistance();
    }
    updateAccelDistance();
    SmartDashboard.putNumber("Velocity Forwards", getVelocityForward());
    SmartDashboard.putNumber("Velocity Sideways", getVelocityStrafe());
    SmartDashboard.putNumber("Distance Forwards", getDistanceForward());
    SmartDashboard.putNumber("Distance Sideways", getDistanceStrafe());
  }

  /**
   * Returns the total current draw of the drive subsystem.
   *
   * @return The drive motors' total current draw (in amps).
   */
  public double getCurrentDraw() {
    return m_frontLeft.getStatorCurrent() +
        m_frontRight.getStatorCurrent() +
        m_rearLeft.getStatorCurrent() +
        m_rearRight.getStatorCurrent();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1] and the linear
   * speeds have no effect on the angular speed.
   *
   * @param xSpeed Speed of the robot in the x direction (forward/backwards).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double squaredX = Math.copySign(xSpeed * xSpeed, xSpeed);
    double squaredY = Math.copySign(ySpeed * ySpeed, ySpeed);

    if (fieldRelative) {
      m_drive.driveCartesian(squaredX, squaredY, rot, m_gyro.getRotation2d());
    } else {
      m_drive.driveCartesian(squaredX, squaredY, rot);
    }
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  /**
   * Resets the acceleration-based velocity and position data.
   * The robot is expected to be stationary when calling this function.
   */
  public void resetAccelDistance() {
    lastAccelMeasureTime = 0;
    lastAccelX = 0;
    lastAccelY = 0;
    accelDistanceX = 0;
    accelDistanceY = 0;
    accelVelocityX = 0;
    accelVelocityY = 0;
  }

  private void updateAccelDistance() {
    // Get acceleration in meters/second^2
    double curAccelX = accelerometer.getZ() * GFORCE_TO_METERS_PER_SECOND_SQUARED;
    double curAccelY = accelerometer.getY() * GFORCE_TO_METERS_PER_SECOND_SQUARED;

    // Don't calculate distance until second measurement
    if (lastAccelMeasureTime == 0) {
      lastAccelMeasureTime = System.currentTimeMillis();
      lastAccelX = curAccelX;
      lastAccelY = curAccelY;
      return;
    }

    // Get average acceleration since last measurement
    double avgAccelX = (curAccelX + lastAccelX) / 2;
    double avgAccelY = (curAccelY + lastAccelY) / 2;
    lastAccelX = curAccelX;
    lastAccelY = curAccelY;

    // Get time delta since last measurement
    long deltaMillis = lastAccelMeasureTime - System.currentTimeMillis();
    double deltaSec = deltaMillis / 1000.0;
    lastAccelMeasureTime = System.currentTimeMillis();

    
    // Calculate current velocities
    accelVelocityX += avgAccelX * deltaSec;
    accelVelocityY += avgAccelY * deltaSec;

    // Determine current (approximate) position
    accelDistanceX += accelVelocityX * deltaSec;
    accelDistanceY += accelVelocityY * deltaSec;
  }

  /**
   * Gets the approximate distance traveled forwards.
   * @return Distance traveled forwards in meters.
   */
  public double getDistanceForward() {
    return accelDistanceY;
  }

  /**
   * Gets the approximate distance traveled sideways.
   * @return Distance traveled sideways in meters.
   */
  public double getDistanceStrafe() {
    return accelDistanceX;
  }

  /**
   * Gets the approximate velocity in the forwards direction.
   * @return Velocity in the forwards direction, in meters/second.
   */
  public double getVelocityForward() {
    return accelDistanceY;
  }

  /**
   * Gets the approximate velocity in the sideways direction.
   * @return Velocity in the sideways direction, in meters/second.
   */
  public double getVelocityStrafe() {
    return accelDistanceX;
  }
}
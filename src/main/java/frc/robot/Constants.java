// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class DriveConstants {
    public static final int kFrontLeftMotorPort = 15;
    public static final int kRearLeftMotorPort = 13;
    public static final int kFrontRightMotorPort = 12;
    public static final int kRearRightMotorPort = 14;

    public static final double kTrackWidth = 0.5; // 0.5 what?
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.7; // 0.7 what?
    // Distance between centers of front and back wheels on robot

    public static final MecanumDriveKinematics kDriveKinematics =
        new MecanumDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
  }

  public static final class ClimbConstants {
    public static final int kClimbLeftMotorPort = 0;  // TODO: Set port!
    public static final int kClimbRightMotorPort = 0; // TODO: Set port!
    public static final int kClimbEngageMotorPort = 0; // TODO: Set port!
  }

  public static final class ArmConstants {
    public static final int kArmRotateMotorPort = 0; // TODO: Set port!
    public static final int kArmExtendMotorPort = 0; // TODO: Set port!
    public static final int kArmIntakeMotorPort = 0; // TODO: Set port!

    public static final int kWristPitchServoPort = 0; // TODO: Set port!
    public static final int kWristDiffServoPort = 0; // TODO: Set port!
  }

  public static final class LEDConstants {
    public static final int kLedDataDigitalPort = 0; // TODO: Set port!
  }
}
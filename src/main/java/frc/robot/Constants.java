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

 /*
 * IF CURRENT CONTROL IS NEEDED ON PINCHER, SWAP TALON FOR VICTOR AND RE-IMPLEMENT IT!!!
  * 7 talon spx (4 drive, 2 arm (extend, rotate), 1 climb (pull up,right))
  * 2 sparkMax (1 intake, 1 climb (pull up,left))
  * 1 victor spx (1 climb (pincher))
  * 2 victor sp
 */

 /* TODO
  * 1. Verify Mapping
  * 2. Tune arm (extend, rotate) PID loops
  * 4. Measure and set arm positions (rotation + extension)
  * 5. Wrist control
  * 6. Redo control scheme, too many buttons
  * 7. LEDs
  * 8. Vision?
  */

public final class Constants {

  public static final int kTimeoutMs = 30;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class LEDConstants {
    public static final int kLedDataDigitalPort = 0;
  }

  public static final class DriveConstants {
    public static final int kFrontLeftMotorPort = 14;
    public static final int kRearLeftMotorPort = 12;
    public static final int kFrontRightMotorPort = 13;
    public static final int kRearRightMotorPort = 15;

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

  public static final class ArmConstants {
    public static final int kArmRotateMotorPort = 0; // Cim + Talon
    public static final int kArmExtendMotorPort = 0; // Bag + Talon
    public static final int kArmIntakeMotorPort = 20; // Neo 550 + Spark
    public static final int kWristPitchServoPort = 0;
    public static final int kWristDiffServoPort = 0;

    // Engage PID
    public static final int kPIDLoopIdx = 0;

    //Extention PID 
    public static final double kExtendP = 0;
    public static final double kExtendI = 0;
    public static final double kExtendD = 0;
    public static final double kExtendF = 0;

    //Rotation PID 
    public static final double kRotateP = 0;
    public static final double kRotateI = 0;
    public static final double kRotateD = 0;
    public static final double kRotateF = 0;

    //Encoder 
    public final static int kSensorUnitsPerRotation = 4096;

    // Intake
    public final static double kIntakeSpeed = 0.8;
  }

  public static final class ArmPositions {
    public static final int kExtendInPos = 0; // FIND REAL VALUES
    public static final int kExtendOutPos = 5;

    public static final int kRotateGroundPos = 0;
    public static final int kRotateCoralOne = 5;
    public static final int kRotateCoralTwo = 10;
    public static final int kRotateCoralThree = 15;
    public static final int kRotateCoralFour = 20;
  }

  public static final class ClimbConstants {
    public static final int kRightClimbMotorPort = 11; // Cim + Talon
    public static final int kLeftClimbMotorPort = 17; // Cim + Spark
    public static final int kEngageClimbMotorPort = 16; // Bag + Victor

    // current constants
    public static final double kClimbPercentEnabled = 0.2;
    public static final double kClimbPercentDisabled = 0.0;
  }
}
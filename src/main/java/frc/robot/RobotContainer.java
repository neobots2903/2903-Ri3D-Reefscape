package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();

  private final double DEADZONE_THRESH = 0.1;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_operatorController = 
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_driveSubsystem.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                m_driveSubsystem.drive(
                    deadzone(m_driverController.getLeftY()),
                    deadzone(m_driverController.getLeftX()),
                    -deadzone(m_driverController.getRightX()),
                    false),
            m_driveSubsystem));

            
    //DRIVER CONTROLLER

        // Drive at half speed when the right bumper is held
    m_driverController.rightBumper()
      .onTrue(new InstantCommand(() -> m_driveSubsystem.setMaxOutput(0.5)))
      .onFalse(new InstantCommand(() -> m_driveSubsystem.setMaxOutput(1)));

    //OPERATOR CONTROLLER

        // Climb up cage when Y is held
    m_operatorController.y()
        .onTrue(new InstantCommand(() -> m_climbSubsystem.setClimbPower(0.85)))
        .onFalse(new InstantCommand(() -> m_climbSubsystem.setClimbPower(0)));

        // X button toggles the climb engagement mechanism. Default false.
    m_operatorController.x()
        .onTrue(new InstantCommand(() -> m_climbSubsystem.setEngagePower(
          m_climbSubsystem.getEngagedStatus() ? ClimbConstants.kClimbCurrentDisabled : ClimbConstants.kClimbCurrentEnabled
          )));

        // A button to intake
    m_operatorController.a()
        .whileTrue(new InstantCommand(() -> m_armSubsystem.SetIntakeSpeed(ArmConstants.kIntakeSpeed)));
    
        // B button to spit out
    m_operatorController.b()
        .whileTrue(new InstantCommand(() -> m_armSubsystem.SetIntakeSpeed(-ArmConstants.kIntakeSpeed)));

        // Left D-Pad to reach Coral L1
    m_operatorController.povLeft()
        .onTrue(new InstantCommand(() -> m_armSubsystem.SetRotationPos(ArmPositions.kRotateCoralOne)));

        // Up D-Pad to reach Coral L2
    m_operatorController.povUp()
    .onTrue(new InstantCommand(() -> m_armSubsystem.SetRotationPos(ArmPositions.kRotateCoralTwo)));

        // Right D-Pad to reach Coral L3
    m_operatorController.povRight()
    .onTrue(new InstantCommand(() -> m_armSubsystem.SetRotationPos(ArmPositions.kRotateCoralThree)));

        // Down D-Pad to reach Coral L4
    m_operatorController.povDown()
    .onTrue(new InstantCommand(() -> m_armSubsystem.SetRotationPos(ArmPositions.kRotateCoralFour)));

        //back to reach ground position
    m_operatorController.back()
      .onTrue(new InstantCommand(() -> m_armSubsystem.SetRotationPos(ArmPositions.kRotateGroundPos)));

        // LB to retract arm
    m_operatorController.leftBumper()
    .onTrue(new InstantCommand(() -> m_armSubsystem.SetExtensionPos(ArmPositions.kExtendInPos)));

        // RB to retract arm
    m_operatorController.rightBumper()
    .onTrue(new InstantCommand(() -> m_armSubsystem.SetExtensionPos(ArmPositions.kExtendOutPos)));
  }
  
  private double deadzone(double val) {
    return (Math.abs(val) > DEADZONE_THRESH) ? val : 0;
  }

}
package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final double DEADZONE_THRESH = 0.1;
  
  /*command to run shooter then actuate solenoid 
  * step 1: run shooter wheels
  * step 2: wait for shooter to reach speed
  * step 3: actuate solenoid
  * step 4: wait for note to shoot
  * step 5: stop shooter wheels and retract solenoid
  */

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController m_driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);

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
                    true),
            m_driveSubsystem));

        // Drive at half speed when the right bumper is held
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> m_driveSubsystem.setMaxOutput(0.5)))
        .onFalse(new InstantCommand(() -> m_driveSubsystem.setMaxOutput(1)));
  
  }
  
  private double deadzone(double val) {
    return (Math.abs(val) > DEADZONE_THRESH) ? val : 0;
  }

}
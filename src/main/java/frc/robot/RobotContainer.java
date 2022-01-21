package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.drive.auto;
import frc.robot.commands.auto.Simple;
import frc.robot.commands.sensors.ResetPose;
import frc.robot.commands.sensors.ResetRotation;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.utils.controllers.CSPController;
import frc.robot.utils.controllers.CSPController.Scaling;
import frc.robot.utils.math.Derivative;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private Swerve swerve = Swerve.getInstance();

  private CSPController pilot = new CSPController(0);

  private SendableChooser<SequentialCommandGroup> autoChooser = new SendableChooser<SequentialCommandGroup>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Set the default commands
    setDefaultCommands();
    // Configure the button bindings
    configureButtonBindings();
    // Add options to the auto chooser
    addChooser();
  }

  private void setDefaultCommands() {
    swerve.setDefaultCommand(new RunCommand(() -> swerve.drive(
      pilot.getLeftY(Scaling.CUBED),
      pilot.getLeftX(Scaling.CUBED),
      pilot.getRightX(Scaling.CUBED),
      pilot.getRightBumper()),
      swerve)
    );  
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {
    SmartDashboard.putData("Reset Position", new ResetPose());
    SmartDashboard.putData("Reset Rotation", new ResetRotation());

    pilot.setRumble(Derivative.getRate(Derivative.getRate(swerve.getVelocity())));
  }

  private void addChooser() {
    autoChooser.setDefaultOption("Do nothing", new SequentialCommandGroup());
    autoChooser.addOption("Simple", new Simple());

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
}

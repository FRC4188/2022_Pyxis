package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.sensors.ResetPose;
import frc.robot.commands.sensors.ResetRotation;
import frc.robot.commands.turret.SetToAngle;
import frc.robot.commands.turret.TurretPower;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utils.CspController;
import frc.robot.utils.CspController.Scaling;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private Swerve swerve = Swerve.getInstance();
  private Turret turret = Turret.getInstance();

  private CspController pilot = new CspController(0);

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
    );  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {
    SmartDashboard.putData("Reset Position", new ResetPose());
    SmartDashboard.putData("Reset Rotation", new ResetRotation());
    
    pilot.getDpadLeftButtonObj().whileHeld(new RunCommand(() -> turret.set(0.5))).whenReleased(new RunCommand(() -> turret.set(0.0)));
    pilot.getDpadRightButtonObj().whileHeld(new RunCommand(() -> turret.set(-0.5))).whenReleased(new RunCommand(() -> turret.set(0.0)));
    

    pilot.getAButtonObj().whenPressed(new SetToAngle(30));
    pilot.getBButtonObj().whenPressed(new SetToAngle(30));




  }

  private void addChooser() {
    autoChooser.setDefaultOption("Do nothing", new SequentialCommandGroup());
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

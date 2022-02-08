package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.SpinShooter;
import frc.robot.commands.turret.SetToAngle;
import frc.robot.commands.turret.TrackTarget;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utils.CSPController;
import frc.robot.utils.CSPController.Scaling;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private Swerve swerve = Swerve.getInstance();
  private Turret turret = Turret.getInstance();
  private Shooter shooter = Shooter.getInstance();

  private CSPController pilot = new CSPController(0);

  private SendableChooser<SequentialCommandGroup> autoChooser = new SendableChooser<SequentialCommandGroup>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Set the default commands
    setDefaultCommands();
    // Configure the button bindings
    configureButtonBindings();

    smartdashboardButtons();
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

    turret.setDefaultCommand(new TrackTarget(true));
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {
    pilot.getDpadLeftButtonObj().whileHeld(new RunCommand(() -> turret.set(0.2), turret)).whenReleased(new RunCommand(() -> turret.set(0.0), turret));
    pilot.getDpadRightButtonObj().whileHeld(new RunCommand(() -> turret.set(-0.2), turret)).whenReleased(new RunCommand(() -> turret.set(0.0), turret));

    pilot.getXButtonObj().whenPressed(new SetToAngle(30));
    pilot.getYButtonObj().whenPressed(new SetToAngle(0));

    pilot.getBButtonObj().whileHeld(new TrackTarget(false));

    pilot.getRbButtonObj().whenPressed(new SpinShooter(() -> SmartDashboard.getNumber("Set Velocity", 0.0)))
      .whenReleased(new SpinShooter(() -> 0.0));
    
  }

  private void smartdashboardButtons() {
    SmartDashboard.putData("Set Turret PIDs", new InstantCommand(() -> turret.setPIDs(
      SmartDashboard.getNumber("Set P", 0),
      SmartDashboard.getNumber("Set I", 0),
      SmartDashboard.getNumber("Set D", 0)
    ), turret));

    SmartDashboard.putData("Set Turret Angle", new SetToAngle(
      SmartDashboard.getNumber("Set Angle", 0.0)
    ));


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

package frc.robot;

import java.io.File;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.drive.auto;
import frc.robot.commands.ShooterVelocity;
import frc.robot.commands.auto.FiveBall;
import frc.robot.commands.auto.Simple;
import frc.robot.commands.auto.ThreeBall;
import frc.robot.commands.auto.TwoBall;
import frc.robot.commands.sensors.ResetPose;
import frc.robot.commands.sensors.ResetRotation;
import frc.robot.planning.Trajectories;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.sensors.Sensors;
import frc.robot.utils.controllers.CSPController;
import frc.robot.utils.controllers.CSPController.Scaling;
import frc.robot.utils.math.Derivative;

/**
 * Controls the robot, holding commands, button bindings, and auto routines.
 */
public class RobotContainer {

  // Auto chooser initialization
  private final SendableChooser<SequentialCommandGroup> autoChooser = new SendableChooser<SequentialCommandGroup>();

  private CSPController pilot = new CSPController(0);

  private Swerve swerve = Swerve.getInstance();
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    CommandScheduler.getInstance().registerSubsystem(swerve, Sensors.getInstance());

    setDefaultCommands();
    configureButtonBindings();
    addChooser();

    SmartDashboard.putNumber("Shooter Set Velocity", 0.0);

    //System.out.println(new File("U").exists());

    //USBLogger.getInstance().setPeriod(0.1);
  }


  /**
   * Method which assigns default commands to different subsystems.
   */
  private void setDefaultCommands() {
    swerve.setDefaultCommand(new RunCommand(() -> swerve.drive(
      pilot.getLeftY(Scaling.SQUARED),
      pilot.getLeftX(Scaling.SQUARED),
      pilot.getRightX(Scaling.SQUARED),
      pilot.getRightBumper()),
      swerve)
    );  
  }

  /**
   * Method which assigns commands to different button actions.
   */
  private void configureButtonBindings() {
    SmartDashboard.putData("Reset Position", new ResetPose());
    SmartDashboard.putData("Reset Rotation", new ResetRotation());

    pilot.getRbButtonObj().whenPressed(new ShooterVelocity(() -> SmartDashboard.getNumber("Shooter Set Velocity", 0.0)))
      .whenReleased(new ShooterVelocity(() -> 0.0));

    //pilot.setRumble(Derivative.getRate(Derivative.getRate(swerve.getVelocity())));
  }

  private void addChooser() {
    autoChooser.setDefaultOption("Do nothing", new SequentialCommandGroup());
    autoChooser.addOption("Simple", new Simple());
    autoChooser.addOption("Two Ball Auto", new TwoBall());
    autoChooser.addOption("Three Ball Auto", new ThreeBall());
    autoChooser.addOption("Five Ball Auto", new FiveBall());

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command autoCommand = autoChooser.getSelected();

    return autoCommand;
  }
}

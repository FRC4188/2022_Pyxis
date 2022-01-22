package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.TestAuto;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.commands.drive.SoftToAngle;
import frc.robot.commands.intake.SpinIntake;
import frc.robot.commands.sensors.ResetPose;
import frc.robot.commands.sensors.ResetRotation;
import frc.robot.planning.Trajectories;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.sensors.Sensors;
import frc.robot.utils.CSPController;
import frc.robot.utils.CSPController.Scaling;

/**
 * Controls the robot, holding commands, button bindings, and auto routines.
 */
public class RobotContainer {

  // Auto chooser initialization
  private final SendableChooser<SequentialCommandGroup> autoChooser = new SendableChooser<SequentialCommandGroup>();

  private Swerve swerve = Swerve.getInstance();
  
  private Intake intake = Intake.getInstance();

  private CSPController pilot = new CSPController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    CommandScheduler.getInstance().registerSubsystem(swerve, Sensors.getInstance());

    setDefaultCommands();
    configureButtonBindings();
    putChooser();

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
    
    //pilot.getAButtonObj().whenPressed();

    SmartDashboard.putData(new ResetPose());
    SmartDashboard.putData(new ResetRotation());
    SmartDashboard.putData(new InstantCommand(() -> intake.set(
      SmartDashboard.getNumber("Test Motor Power", 0)
    ), intake ));
  }

  //intake commands
/* // Intake commands.
    pilot.getAButtonObj().whenPressed(new AutoIntake(true)).whenReleased(new AutoIntake(false));
    pilot
        .getBButtonObj()
        .whenPressed(new SpinIntake(-0.5, true))
        .whenReleased(new SpinIntake(0.0, false)); */


  private void putChooser() {
    autoChooser.setDefaultOption("Do Nothing", new SequentialCommandGroup());
    autoChooser.addOption("Test Auto", new TestAuto());
    //autoChooser.addOption("Steal Auto", new SequentialCommandGroup(new FollowTrajectory(Trajectories.steal.collect, new Rotation2d())));

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

package frc.robot;

import org.ejml.dense.row.decomposition.eig.watched.WatchedDoubleStepQREigen_DDRM;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.InterruptSubsystem;
import frc.robot.commands.auto.FiveBall;
import frc.robot.commands.auto.ThreeBall;
import frc.robot.commands.auto.TwoBall;
import frc.robot.commands.climber.ActivePosition;
import frc.robot.commands.climber.TestBrakes;
import frc.robot.commands.climber.TestPassive;
import frc.robot.commands.groups.MonkeyBar;
import frc.robot.commands.groups.AutoIntake;
import frc.robot.commands.groups.AutoShoot;
import frc.robot.commands.intake.TestPistons;
import frc.robot.commands.sensors.ResetPose;
import frc.robot.commands.sensors.ResetRotation;
import frc.robot.commands.trigger.PushTrigger;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.sensors.Sensors;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.trigger.Trigger;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utils.controllers.CSPController;
import frc.robot.utils.controllers.CSPController.Scaling;

/**
 * Controls the robot, holding commands, button bindings, and auto routines.
 */
public class RobotContainer {

  // Auto chooser initialization
  private final SendableChooser<SequentialCommandGroup> autoChooser = new SendableChooser<SequentialCommandGroup>();

  private CSPController pilot = new CSPController(0);
  private CSPController copilot = new CSPController(1);

  private Swerve swerve = Swerve.getInstance();
  private Climber climber = Climber.getInstance();
  private Intake intake = Intake.getInstance();
  private Indexer indexer = Indexer.getInstance();
  private Trigger trigger = Trigger.getInstance();
  private Turret turret = Turret.getInstance();
  private Shooter shooter = Shooter.getInstance();
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    CommandScheduler.getInstance().registerSubsystem(swerve, Sensors.getInstance());
    SmartDashboard.putNumber("trigger voltage", 0.0);
    SmartDashboard.putNumber("indexer voltage", 0.0);
    setDefaultCommands();
    configureButtonBindings();
    addChooser();
  }


  /**
   * Method which assigns default commands to different subsystems.
   */
  private void setDefaultCommands() {
    swerve.setDefaultCommand(new RunCommand(() -> swerve.drive(
      pilot.getLeftY(Scaling.CUBED),
      pilot.getLeftX(Scaling.CUBED),
      pilot.getRightX(Scaling.CUBED),
      pilot.getRightBumper()),
      swerve)
    );
    //climber.setDefaultCommand(new ActiveVolts(() -> pilot.getLeftY(Scaling.LINEAR) * 12.0, () -> pilot.getRightY(Scaling.LINEAR) * 12.0));

  }

  /**
   * Method which assigns commands to different button actions.
   */
  private void configureButtonBindings() {
    SmartDashboard.putData("Reset Position", new ResetPose());
    SmartDashboard.putData("Reset Rotation", new ResetRotation());

    /*//Competition Bindings
    pilot.getAButtonObj()
      .whenPressed(new AutoIntake())
      .whenReleased(new InterruptSubsystem(intake, indexer));
    
    pilot.getBButtonObj()
      .whenPressed(new AutoShoot())
      .whenReleased(new InterruptSubsystem(shooter, indexer, intake));
    
    pilot.getYButtonObj()
      .whenPressed(new AutoClimb());
    
    pilot.getRbButtonObj()
      .whenPressed(new ActivePosition(Constants.climber.MAX_HEIGHT));
    
    pilot.getLbButtonObj()
      .whenPressed(new ActivePosition(0.0));
    */

    //Testing Bindings
    pilot.getAButtonObj()
      .whileHeld(new AutoIntake())
      .whenReleased(new InterruptSubsystem(indexer, intake));

    pilot.getBButtonObj()
      .whileHeld(new PushTrigger(8.0))
      .whenReleased(new InterruptSubsystem(trigger));

    pilot.getRbButtonObj()
      .whenPressed(new ActivePosition(Constants.climber.MAX_HEIGHT));

    pilot.getLbButtonObj()
      .whenPressed(new ActivePosition(0.0));

    pilot.getYButtonObj()
      .whenPressed(new MonkeyBar());

    pilot.getDpadUpButtonObj()
      .whenPressed(new TestPassive());
    pilot.getDpadLeftButtonObj()
      .whenPressed(new TestBrakes());
    pilot.getDpadRightButtonObj()
      .whenPressed(new TestPistons());
  }

  private void addChooser() {
    autoChooser.setDefaultOption("Do nothing", new SequentialCommandGroup());
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

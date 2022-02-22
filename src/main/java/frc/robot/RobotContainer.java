package frc.robot;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.FiveBall;
import frc.robot.commands.auto.ThreeBall;
import frc.robot.commands.auto.TwoBall;
import frc.robot.commands.climber.ActivePull;
import frc.robot.commands.climber.ActivePush;
import frc.robot.commands.climber.ActiveVolts;
import frc.robot.commands.climber.PassivePull;
import frc.robot.commands.climber.PassivePush;
import frc.robot.commands.climber.TestBrakes;
import frc.robot.commands.climber.TestPassive;
import frc.robot.commands.groups.AutoClimb;
import frc.robot.commands.indexer.TestIndexer;
import frc.robot.commands.intake.TestIntake;
import frc.robot.commands.intake.TestPistons;
import frc.robot.commands.sensors.ResetPose;
import frc.robot.commands.sensors.ResetRotation;
import frc.robot.commands.trigger.TestTrigger;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.sensors.Sensors;
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
  //private Turret turret = Turret.getInstance();
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

    pilot.getAButtonObj()
    .whileHeld(new TestIndexer());

    pilot.getXButtonObj()
      .whileHeld(new TestIntake());

    pilot.getBButtonObj().whileHeld(new TestTrigger());

    //pilot.getYButtonObj().whenPressed(new TestPassive());
    pilot.getRbButtonObj().whenPressed(new ActivePush());
    pilot.getLbButtonObj().whenPressed(new ActivePull());

    pilot.getYButtonObj().whenPressed(new AutoClimb());

    pilot.getDpadUpButtonObj().whenPressed(new TestPassive());
    pilot.getDpadLeftButtonObj().whenPressed(new TestBrakes());
    pilot.getDpadRightButtonObj().whenPressed(new TestPistons());
    /*
    pilot.getDpadLeftButtonObj().whileHeld(new RunCommand(() -> turret.set(0.2), turret)).whenReleased(new RunCommand(() -> turret.set(0.0), turret));
    pilot.getDpadRightButtonObj().whileHeld(new RunCommand(() -> turret.set(-0.2), turret)).whenReleased(new RunCommand(() -> turret.set(0.0), turret));

    pilot.getBButtonObj().whileHeld(new TrackTarget(true)).whenReleased(new TrackTarget(false));

    pilot.getRbButtonObj().whileHeld(new AutoShoot()).whenReleased(new ParallelCommandGroup(new ShooterVelocity(()->0.0), new InstantCommand(() -> turret.set(0.0))));*/
    //pilot.getRbButtonObj().whenPressed(new ShooterVelocity(() -> SmartDashboard.getNumber("Shooter Set Velocity", 0.0))).whenReleased(new ShooterVelocity(() -> 0.0));

    copilot.getDpadLeftButtonObj().whenPressed(new PassivePull());
    copilot.getDpadRightButtonObj().whenPressed(new PassivePush());
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

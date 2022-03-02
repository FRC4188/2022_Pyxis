package frc.robot;

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
import frc.robot.commands.climber.FindZeros;
import frc.robot.commands.climber.TestPassive;
import frc.robot.commands.groups.MonkeyBar;
import frc.robot.commands.intake.TestPistons;
import frc.robot.commands.groups.AutoIntake;
import frc.robot.commands.groups.AutoShoot;
import frc.robot.commands.sensors.ResetPose;
import frc.robot.commands.sensors.ResetRotation;
import frc.robot.commands.shooter.DashHood;
import frc.robot.commands.shooter.FindHoodZeros;
import frc.robot.commands.shooter.ShooterVelocity;
import frc.robot.commands.trigger.PushTrigger;
import frc.robot.commands.turret.TrackTarget;
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
    SmartDashboard.putData("Set Shooter Voltage Command", new RunCommand(() -> shooter.setVolts(SmartDashboard.getNumber("Hood Set Voltage", 0.0))));
    SmartDashboard.putData("Hood Set Angle Command", new DashHood());
    SmartDashboard.putData("Find Hood Zero", new FindHoodZeros());


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
    pilot.getBButtonObj()
      .whileHeld(new AutoIntake())
      .whenReleased(new InterruptSubsystem(indexer, intake));

    pilot.getXButtonObj()
      .whileHeld(new ShooterVelocity(() -> SmartDashboard.getNumber("Shooter Set Velocity", 0.0)))
      .whenReleased(new InterruptSubsystem(shooter));
    
    pilot.getAButtonObj()
      .whenPressed(new TrackTarget(true))
      .whenReleased(new InterruptSubsystem(turret));
    /*
      .whenPressed(new SpinIntake(-12.0))
      .whenReleased(new InterruptSubsystem(intake));
      */
    
    pilot.getYButtonObj()
      .whenPressed(new AutoShoot())
      //.whenPressed(new PushTrigger(12.0))
      .whenReleased(new InterruptSubsystem(shooter, trigger));

    pilot.getRbButtonObj()
      .whenPressed(new ActivePosition(Constants.climber.MAX_HEIGHT));

    pilot.getLbButtonObj()
      .whenPressed(new ActivePosition(0.0));

    pilot.getDpadUpButtonObj()
      .whenPressed(new MonkeyBar());
    
    pilot.getDpadDownButtonObj()
      .whenPressed(new FindZeros().andThen(new ActivePosition(0.0)));

    pilot.getDpadRightButtonObj()
      .whenPressed(new TestPassive());
    pilot.getDpadLeftButtonObj()
      .whenPressed(new TestPistons());

      /*
    pilot.getDpadRightButtonObj()
      .whenPressed(new RunCommand(() -> turret.set(0.2), turret))
      .whenReleased(new InterruptSubsystem(turret));
    
    pilot.getDpadLeftButtonObj()
      .whenPressed(new RunCommand(() -> turret.set(-0.2), turret))
      .whenReleased(new InterruptSubsystem(turret));
      */
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

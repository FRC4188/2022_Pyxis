package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.InterruptSubsystem;
import frc.robot.commands.auto.FiveBall;
import frc.robot.commands.auto.ThreeBall;
import frc.robot.commands.auto.TwoBall;
import frc.robot.commands.climber.ActivePosition;
import frc.robot.commands.climber.FindZeros;
import frc.robot.commands.climber.ToggleBrakes;
import frc.robot.commands.climber.TogglePassive;
import frc.robot.commands.groups.MonkeyBar;
import frc.robot.commands.indexer.SpinIndexer;
import frc.robot.commands.intake.SpinIntake;
import frc.robot.commands.intake.ToggleIntakePistons;
import frc.robot.commands.groups.AutoIntake;
import frc.robot.commands.groups.AutoShoot;
import frc.robot.commands.sensors.ResetPose;
import frc.robot.commands.sensors.ResetRotation;
import frc.robot.commands.shooter.FindHoodZeros;
import frc.robot.commands.shooter.HoodAngle;
import frc.robot.commands.shooter.ShooterVelocity;
import frc.robot.commands.trigger.PushTrigger;
import frc.robot.commands.turret.SetToAngle;
import frc.robot.commands.turret.TrackTarget;
import frc.robot.commands.turret.WrapAround;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.sensors.Sensors;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.trigger.PreShooter;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  private PreShooter trigger = PreShooter.getInstance();
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
      pilot.getRightX(Scaling.CUBED)),
      swerve)
    );

    new Trigger(() -> turret.getPosition() > Constants.turret.MAX_ANGLE).whenActive(
      new SequentialCommandGroup(
        new SetToAngle(Constants.turret.MAX_ANGLE - 180.0),
        new WrapAround(true)
    ));
    new Trigger(() -> turret.getPosition() > Constants.turret.MIN_ANGLE).whenActive(
      new SequentialCommandGroup(
        new SetToAngle(Constants.turret.MIN_ANGLE + 180.0),
        new WrapAround(false)
    ));  }

  /**
   * Method which assigns commands to different button actions.
   */
  private void configureButtonBindings() {
    SmartDashboard.putData("Reset Position", new ResetPose());
    SmartDashboard.putData("Reset Rotation", new ResetRotation());
    SmartDashboard.putData("Set Shooter Voltage Command", new RunCommand(() -> shooter.setVolts(SmartDashboard.getNumber("Hood Set Voltage", 0.0))));
    SmartDashboard.putData("Hood Set Angle Command", new HoodAngle(() -> SmartDashboard.getNumber("Hood Set Angle", 0.0)));
    SmartDashboard.putData("Find Hood Zero", new FindHoodZeros());
    SmartDashboard.putData("Zero Climber", new FindZeros().andThen(new ActivePosition(0.0)));
    SmartDashboard.putData("Set Shooter Velocity Command", new ShooterVelocity(() -> SmartDashboard.getNumber("Shooter Set Velocity", 0.0)));
    SmartDashboard.putData("Turret to -90", new SetToAngle(-90.0));
    SmartDashboard.putData("Turret to 0.0", new SetToAngle(0.0));
    SmartDashboard.putData("Toggle Climber Brakes", new ToggleBrakes());


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
    
    pilot.getAButtonObj()
      .whileHeld(new TrackTarget())
      .whenReleased(new InterruptSubsystem(turret));
    
    pilot.getYButtonObj()
      //.whenPressed(new PushTrigger(12.0))
      .whenPressed(new AutoShoot())
      .whenReleased(new InterruptSubsystem(shooter, trigger, indexer));
    
    pilot.getXButtonObj()
      .whenPressed(new ParallelCommandGroup(new SpinIndexer(-8.0), new SpinIntake(-12.0)))
      .whenReleased(new InterruptSubsystem(indexer, trigger));
    
    pilot.getRbButtonObj()
      .whenPressed(new ToggleIntakePistons());
    
    pilot.getLbButtonObj()
      .whenPressed(new TogglePassive());

    pilot.getDpadUpButtonObj()
      .whenPressed(new ActivePosition(Constants.climber.MAX_HEIGHT));

    pilot.getDpadDownButtonObj()
      .whenPressed(new ActivePosition(0.0));

    pilot.getStartButtonObj()
      .whenPressed(new MonkeyBar());

    pilot.getBackButtonObj()
      .whenPressed(new TogglePassive());

    pilot.getDpadRightButtonObj()
      .whenPressed(new InstantCommand(() -> turret.set(0.2), turret))
      .whenReleased(new InstantCommand(() -> turret.set(0.0), turret));
    
    pilot.getDpadLeftButtonObj()
      .whenPressed(new InstantCommand(() -> turret.set(-0.2), turret))
      .whenReleased(new InstantCommand(() -> turret.set(0.0), turret));
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

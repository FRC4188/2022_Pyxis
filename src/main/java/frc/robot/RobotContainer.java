package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.shooter;
import frc.robot.Constants.turret;
import frc.robot.commands.InterruptSubsystem;
import frc.robot.commands.auto.FiveBall;
import frc.robot.commands.auto.FiveBallPlus;
import frc.robot.commands.auto.FourBall;
import frc.robot.commands.auto.GenericTwoBall;
import frc.robot.commands.auto.StealAuto;
import frc.robot.commands.auto.The1771Auto;
import frc.robot.commands.climber.ActivePosition;
import frc.robot.commands.climber.ActiveVolts;
import frc.robot.commands.climber.FindZeros;
import frc.robot.commands.climber.ToggleBrakes;
import frc.robot.commands.climber.TogglePassive;
import frc.robot.commands.groups.MonkeyBar;
import frc.robot.commands.groups.PresetShoot;
import frc.robot.commands.indexer.SpinIndexer;
import frc.robot.commands.intake.SpinIntake;
import frc.robot.commands.intake.ToggleIntakePistons;
import frc.robot.commands.groups.AutoIntake;
import frc.robot.commands.groups.AutoShoot;
import frc.robot.commands.groups.BlindShoot;
import frc.robot.commands.groups.LowerPortShot;
import frc.robot.commands.sensors.ResetPose;
import frc.robot.commands.sensors.ResetRotation;
import frc.robot.commands.shooter.FindHoodZeros;
import frc.robot.commands.shooter.HoodAngle;
import frc.robot.commands.shooter.ShooterVelocity;
import frc.robot.commands.trigger.PushTrigger;
import frc.robot.commands.turret.SetToAngle;
import frc.robot.commands.turret.TrackTarget;
import frc.robot.commands.turret.Hunt;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.sensors.Sensors;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.trigger.PreShooter;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utils.ButtonBox;
import frc.robot.utils.CSPController;
import frc.robot.utils.CSPController.Scaling;

/**
 * Controls the robot, holding commands, button bindings, and auto routines.
 */
public class RobotContainer {

  // Auto chooser initialization
  private final SendableChooser<SequentialCommandGroup> autoChooser = new SendableChooser<SequentialCommandGroup>();

  private CSPController pilot = new CSPController(0);
  private CSPController copilot = new CSPController(1);
  private ButtonBox buttonBox = new ButtonBox(2);

  private Swerve swerve = Swerve.getInstance();
  private Climber climber = Climber.getInstance();
  private Intake intake = Intake.getInstance();
  private Indexer indexer = Indexer.getInstance();
  private PreShooter trigger = PreShooter.getInstance();
  private Turret turret = Turret.getInstance();
  private Shooter shooter = Shooter.getInstance();
  private Hood hood = Hood.getInstance();

  private double lastSetHood = 0.0;
  private double lastSetShooter = 0.0;
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
      pilot.getRightX(Scaling.
      CUBED)),
      swerve)
    );

    //turret.setDefaultCommand(new TrackTarget());

    new Trigger(() -> turret.getPosition() >= Constants.turret.MAX_ANGLE).whenActive(new RunCommand(() -> turret.setVolts(8.0)).withTimeout(0.5).andThen(new Hunt(true)), true);
    new Trigger(() -> turret.getPosition() <= Constants.turret.MIN_ANGLE).whenActive(new RunCommand(() -> turret.setVolts(-8.0)).withTimeout(0.5).andThen(new Hunt(false)), true);
    new Trigger(() -> !Sensors.getInstance().getHasTarget()).whenActive(new SetToAngle(0.0));

    new Trigger(() -> {
      boolean changed = SmartDashboard.getNumber("Shooter Set Velocity", 0.0) != lastSetShooter;
      lastSetShooter = SmartDashboard.getNumber("Shooter Set Velocity", 0.0);
      return changed;
    }).whenActive(new ShooterVelocity(() -> SmartDashboard.getNumber("Shooter Set Velocity", 0.0)));

    new Trigger(() -> {
      boolean changed = SmartDashboard.getNumber("Hood Set Angle", 0.0) != lastSetHood;
      lastSetHood = SmartDashboard.getNumber("Hood Set Angle", 0.0);
      return changed;
    }).whenActive(new HoodAngle(() -> SmartDashboard.getNumber("Hood Set Angle", 0.0)));
  }

  /**
   * Method which assigns commands to different button actions.
   */
  private void configureButtonBindings() {
    SmartDashboard.putData("Reset Position", new ResetPose());
    SmartDashboard.putData("Reset Rotation", new ResetRotation());
    SmartDashboard.putData("Find Hood Zero", new FindHoodZeros());
    SmartDashboard.putData("Zero Climber", new FindZeros().andThen(new ActivePosition(0.0)));
    SmartDashboard.putData("Toggle Climber Brakes", new ToggleBrakes());

    pilot.getBButtonObj()
      .whileHeld(new AutoIntake())
      .whenReleased(new InterruptSubsystem(indexer, intake));
    
    pilot.getAButtonObj()
      .whileHeld(new TrackTarget())
      .whenReleased(new InterruptSubsystem(turret));
    
    pilot.getYButtonObj()
      .whenPressed(new AutoShoot(true))
      .whenReleased(new InterruptSubsystem(shooter, trigger, indexer, hood));
    
    pilot.getXButtonObj()
      .whenPressed(new ParallelCommandGroup(new PushTrigger(-8.0), new SpinIndexer(-8.0), new SpinIntake(-12.0, true)))
      .whenReleased(new InterruptSubsystem(indexer, trigger, intake));
    
    pilot.getRbButtonObj()
      .whenPressed(new LowerPortShot())
      .whenReleased(new InterruptSubsystem(turret, shooter, trigger, indexer));
    
    pilot.getLbButtonObj()
      .whenPressed(new BlindShoot(10.5, 2500.0))
      .whenReleased(new InterruptSubsystem(shooter, hood, turret, trigger));

    pilot.getDpadUpButtonObj()
      .whenPressed(new ActivePosition(Constants.climber.MAX_HEIGHT));

    pilot.getDpadDownButtonObj()
    .whenPressed(new ActivePosition(0.0));
    
    pilot.getStartButtonObj()
      .whenPressed(new MonkeyBar());

    pilot.getBackButtonObj()
      .whenPressed(new ResetPose());

    pilot.getDpadRightButtonObj()
      .whenPressed(new RunCommand(() -> turret.setVolts(3.0), turret))
      .whenReleased(new InstantCommand(() -> turret.set(0.0), turret));
    
    pilot.getDpadLeftButtonObj()
      .whenPressed(new RunCommand(() -> turret.setVolts(-3.0), turret))
      .whenReleased(new InstantCommand(() -> turret.set(0.0), turret));

    copilot.getAButtonObj()
      .whenPressed(new SpinIntake(12.0, false))
      .whenReleased(new InterruptSubsystem(intake));
    
    copilot.getBButtonObj()
      .whenPressed(new SpinIndexer(8.0))
      .whenReleased(new InterruptSubsystem(indexer));
    
    copilot.getXButtonObj()
      .whenPressed(new PushTrigger(12.0))
      .whenReleased(new InterruptSubsystem(trigger));
    
    copilot.getDpadUpButtonObj()
      .whenPressed(new SetToAngle(0.0));

    copilot.getDpadLeftButtonObj()
      .whenPressed(new SetToAngle(-90.0));

    copilot.getDpadDownButtonObj()
      .whenPressed(new SetToAngle(-180.0));

    copilot.getDpadRightButtonObj()
      .whenPressed(new SetToAngle(-270.0));

    copilot.getLbButtonObj()
      .whenPressed(new TogglePassive());
      
    copilot.getLbButtonObj()
        .whenPressed(new RunCommand(() -> hood.setVolts(2.0), hood))
        .whenReleased(new RunCommand(() -> hood.setVolts(0.0), hood));
    
    buttonBox.getButton1Obj()
      .whenPressed(new BlindShoot(10.5, 2500.0))
      .whenReleased(new InterruptSubsystem(shooter, hood, turret, trigger));

    buttonBox.getButton2Obj()
      .whenPressed(new PresetShoot(16.6, 2630.0))
      .whenReleased(new InterruptSubsystem(shooter, hood, turret, trigger));
  }

  private void addChooser() {
    autoChooser.setDefaultOption("Do nothing", new SequentialCommandGroup());
    autoChooser.addOption("Five Ball Auto", new FiveBall());
    autoChooser.addOption("Four Ball Auto", new FourBall());
    autoChooser.addOption("Generic Two Ball Auto", new GenericTwoBall());
    autoChooser.addOption("Five Ball Plus Auto", new FiveBallPlus());
    autoChooser.addOption("1771 Auto", new The1771Auto());
    autoChooser.addOption("Steal Auto", new StealAuto());

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

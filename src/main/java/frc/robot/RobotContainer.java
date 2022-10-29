package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.shooter;
import frc.robot.Constants.turret;
import frc.robot.commands.InterruptSubsystem;
import frc.robot.commands.auto.FiveBall;
import frc.robot.commands.auto.FiveBallPlus;
import frc.robot.commands.auto.FourBall;
import frc.robot.commands.auto.GenericTwoBall;
import frc.robot.commands.auto.HoardingAuto;
import frc.robot.commands.auto.StealAuto;
import frc.robot.commands.auto.TestOTFRadial;
import frc.robot.commands.auto.The1771Auto;
import frc.robot.commands.climber.ActivePosition;
import frc.robot.commands.climber.ActiveVolts;
import frc.robot.commands.climber.FindZeros;
import frc.robot.commands.climber.ToggleBrakes;
import frc.robot.commands.climber.TogglePassive;
import frc.robot.commands.drive.CrabSet;
import frc.robot.commands.groups.MonkeyBar;
import frc.robot.commands.groups.PresetShoot;
import frc.robot.commands.indexer.LoadBalls;
import frc.robot.commands.indexer.SpinIndexer;
import frc.robot.commands.intake.SpinIntake;
import frc.robot.commands.intake.ToggleIntakePistons;
import frc.robot.commands.groups.AutoIntake;
import frc.robot.commands.groups.AutoShoot;
import frc.robot.commands.groups.BlindShoot;
import frc.robot.commands.groups.LowerPortShot;
import frc.robot.commands.sensors.ResetPose;
import frc.robot.commands.sensors.ResetRotation;
import frc.robot.commands.shooter.HoodAngle;
import frc.robot.commands.shooter.ShooterVelocity;
import frc.robot.commands.trigger.AutoFire;
import frc.robot.commands.trigger.PushTrigger;
import frc.robot.commands.turret.SetToAngle;
import frc.robot.commands.turret.TrackTarget;
import frc.robot.commands.turret.TurretAngleWait;
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

  private Notifier shuffleUpdater = new Notifier(() -> updateShuffle());
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    setDefaultCommands();
    configureButtonBindings();
    addChooser();
    shuffleUpdater.startPeriodic(0.1);
  }

  public void updateShuffle() {
    swerve.smartDashboard();
    hood.updateDashboard();
    shooter.updateDashboard();
    Sensors.getInstance().updateDashboard();
  }

  /**
   * Method which assigns default commands to different subsystems.
   */
  private void setDefaultCommands() {
    swerve.setDefaultCommand(new RunCommand(() -> swerve.drive(
      pilot.getLeftY(Scaling.CUBED),
      pilot.getLeftX(Scaling.CUBED),
      pilot.getRightX(Scaling.SQUARED),
      () -> pilot.getRightBumper()),
      swerve)
      //new CrabSet(0.0, 0.0)
    );
    turret.setDefaultCommand(
    //   /*
    //   new RunCommand(() -> {
    //   double current = turret.getPosition();
    //   double required = -(Sensors.getInstance().getTargetAngle() + 180.0);
    //   double set = current + (required - (current + 180.0) % 360.0 - 180.0);
    //   turret.setAngle(set);
    // }, turret)*/
    new TrackTarget());
    shooter.setDefaultCommand(new ShooterVelocity(() -> Sensors.getInstance().getFormulaRPM()));
    hood.setDefaultCommand(new HoodAngle(() -> Sensors.getInstance().getFormulaAngle()));
    trigger.setDefaultCommand(new AutoFire());
    indexer.setDefaultCommand(new LoadBalls());
    climber.setDefaultCommand(new RunCommand(() -> climber.setActiveVolts(0.0), climber));
    intake.setDefaultCommand(new RunCommand(() -> intake.setVoltage(0.0), intake));

    new Trigger(() -> turret.getPosition() >= Constants.turret.MAX_ANGLE).whenActive(new ParallelDeadlineGroup(
      new TurretAngleWait(turret.getPosition() - 180.0).withTimeout(0.45),
      new RunCommand(() -> turret.setVolts(-12.0))
    ).andThen(new Hunt(true)), false);

    new Trigger(() -> turret.getPosition() <= Constants.turret.MIN_ANGLE).whenActive(new ParallelDeadlineGroup(
      new TurretAngleWait(turret.getPosition() + 180.0).withTimeout(0.45),
      new RunCommand(() -> turret.setVolts(12.0))
    ).andThen(new Hunt(false)), false);

  }

  /**
   * Method which assigns commands to different button actions.
   */
  private void configureButtonBindings() {
    SmartDashboard.putData("Reset Position", new ResetPose());
    SmartDashboard.putData("Reset Rotation", new ResetRotation());
    SmartDashboard.putData("Zero Climber", new FindZeros().andThen(new ActivePosition(0.0)));
    SmartDashboard.putData("Toggle Climber Brakes", new ToggleBrakes());
    SmartDashboard.putData("Set Ball Track PID", new InstantCommand(() -> turret.setTPID(
      SmartDashboard.getNumber("T kP", 0.0), 
      SmartDashboard.getNumber("T kI", 0.0), 
      SmartDashboard.getNumber("T kD", 0.0))));
    SmartDashboard.putData("Prioritize Drivetrain", new ParallelCommandGroup(
      new RunCommand(() -> indexer.set(0.0), indexer),
      new RunCommand(() -> shooter.setVolts(0.0), shooter)));

      
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


    pilot.getBButtonObj()
      .whileHeld(new SpinIntake(8.5))
      .whenReleased(new InterruptSubsystem(intake));
    
    pilot.getYButtonObj()
      .whenPressed(new AutoShoot(true))
      .whenReleased(new InterruptSubsystem(shooter, trigger, indexer, hood));
    
    pilot.getXButtonObj()
      .whenPressed(new ParallelCommandGroup(new PushTrigger(-8.0), new SpinIndexer(-8.0), new SpinIntake(-12.0, true)))
      .whenReleased(new InterruptSubsystem(indexer, trigger, intake));

    pilot.getLbButtonObj()
      .whenPressed(new BlindShoot(10.5, 2500.0))
      .whenReleased(new InterruptSubsystem(shooter, hood, turret, trigger));

    pilot.getDpadUpButtonObj()
      .whenPressed(new ActivePosition(Constants.climber.MAX_HEIGHT + 0.03));

    pilot.getDpadDownButtonObj()
    .whenPressed(new ActivePosition(0.0)).whenActive(new RunCommand(() -> shooter.setVelocity(0), shooter));
    
    pilot.getStartButtonObj()
      .whenPressed(new MonkeyBar());

    pilot.getBackButtonObj()
      .whenPressed(new ResetPose(new Pose2d(0.0, -1.0, new Rotation2d(-Math.PI/2.0))));

    pilot.getDpadRightButtonObj()
      .whenPressed(new SetToAngle(-180.0));

    
    pilot.getDpadLeftButtonObj()
      .whenPressed(new SetToAngle(0.0));

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

    copilot.getBackButtonObj()
    .whenPressed(
      new SequentialCommandGroup(
        new InstantCommand(() -> Sensors.getInstance().setPower(false)),
        new WaitCommand(5.0),
        new InstantCommand(() -> Sensors.getInstance().setPower(true))
      )
    );
      
    copilot.getLbButtonObj()
        .whenPressed(new RunCommand(() -> hood.setVolts(2.0), hood))
        .whenReleased(new RunCommand(() -> hood.setVolts(0.0), hood));
  }

  private void addChooser() {
    autoChooser.setDefaultOption("Do nothing", new SequentialCommandGroup());
    autoChooser.addOption("Five Ball Auto", new FiveBall());
    autoChooser.addOption("Four Ball Auto", new FourBall());
    autoChooser.addOption("Generic Two Ball Auto", new GenericTwoBall());
    autoChooser.addOption("Five Ball Plus Auto", new FiveBallPlus());
    autoChooser.addOption("1771 Auto", new The1771Auto());
    autoChooser.addOption("Steal Auto", new StealAuto());
    autoChooser.addOption("Hoard Auto", new HoardingAuto());
    autoChooser.addOption("TEST OTF RADIAL", new TestOTFRadial());

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

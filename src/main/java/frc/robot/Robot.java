/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climber.ActivePosition;
import frc.robot.commands.climber.FindZeros;
import frc.robot.commands.shooter.FindHoodZeros;
import frc.robot.commands.shooter.HoodAngle;
import frc.robot.commands.tests.BallTrackTest;
import frc.robot.commands.tests.PneumaticsTest;
import frc.robot.commands.tests.ShooterTest;
import frc.robot.commands.tests.SwerveTest;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.sensors.Sensors;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer = new RobotContainer();

  private final SendableChooser<SequentialCommandGroup> testChooser = new SendableChooser<SequentialCommandGroup>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    PneumaticsControlModule pcm = new PneumaticsControlModule();
    pcm.clearAllStickyFaults();
    pcm.close();
    Sensors.getInstance().setLED(false);

    testChooser.setDefaultOption("Do nothing", new SequentialCommandGroup());
    testChooser.addOption("Swerve Test", new SwerveTest());
    testChooser.addOption("Ball Path Test", new BallTrackTest());
    testChooser.addOption("Pneumatics Test", new PneumaticsTest());
    testChooser.addOption("Shooter Test", new ShooterTest());

    SmartDashboard.putData("Test Chooser", testChooser);


    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setFPS(20);
    camera.setResolution(320, 240);
    camera.setExposureAuto();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    //tempManager.run();

    /*ledPanel.set(LEDPanel.SYSTEM.GENERAL, 0, RobotController.getBatteryVoltage() > 12.0 ? LEDPanel.BEHAVIOR.OFF :
                                             RobotController.getBatteryVoltage() > 10.0 ? LEDPanel.BEHAVIOR.BLINK :
                                                                                          LEDPanel.BEHAVIOR.ON);*/

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    Climber.getInstance().setBrake(true);
    Sensors.getInstance().setLED(false);
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    //Sensors.getInstance().setLED(true);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    new FindZeros().andThen(new ActivePosition(0.0)).schedule();
    new HoodAngle(() -> 10.0).withTimeout(0.1).andThen(new FindHoodZeros()).schedule();

    if (RobotController.getBatteryVoltage() < 12.3) DriverStation.reportWarning("Battery voltage too low; please change battery.", false);
    //m_robotContainer.resetRobot();

    //Sensors.getInstance().setLED(true);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    //tempManager.run();
    //bop.run();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    testChooser.getSelected().schedule();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}

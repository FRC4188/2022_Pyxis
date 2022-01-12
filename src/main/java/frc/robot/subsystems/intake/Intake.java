// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MusicPlayer;

public class Intake extends SubsystemBase {

  private static Intake instance;

  public static synchronized Intake getInstace() {
    if (instance == null) instance = new Intake();
    return instance;
  }

  private FourBar fourBar = new FourBar(0);
  private WPI_TalonFX intakeMotor = new WPI_TalonFX(14);

  private Notifier shuffle = new Notifier(() -> shuffle());

  /** Creates a new Intake. */
  private Intake() {
    CommandScheduler.getInstance().registerSubsystem(this);

    intakeMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor.configOpenloopRamp(Constants.intake.RAMP_RATE);
    intakeMotor.setInverted(true);

    shuffle.startPeriodic(0.2);

    MusicPlayer.getInstance().addMotor(intakeMotor);
  }

  @Override
  public void periodic() {}

  private void shuffle() {
    SmartDashboard.putBoolean("Intake Solenoid", getRaised());
  }

  public void set(double power) {
    intakeMotor.set(power);
  }

  public void setRaised(boolean raised) {
    fourBar.setRaised(!raised);
  }

  public void relax() {
    fourBar.relax();
  }

  public boolean getRaised() {
    return fourBar.getRaised();
  }

  public void toggleRaised() {
    fourBar.setRaised(!getRaised());
  }

  public double getTemperature() {
    return intakeMotor.getTemperature();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MusicPlayer;

public class Hopper extends SubsystemBase {

  private static Hopper instance;

  public static synchronized Hopper getInstance() {
    if (instance == null) instance = new Hopper();
    return instance;
  }

  private DigitalInput breaker1 = new DigitalInput(0);
  private DigitalInput breaker2 = new DigitalInput(1);
  private DigitalInput breaker3 = new DigitalInput(2);
  private DigitalInput breaker4 = new DigitalInput(3);

  private WPI_TalonFX hopperMotor = new WPI_TalonFX(9);
  Notifier shuffle = new Notifier(() -> updateShuffle());

  /** Creates a new Hopper. */
  public Hopper() {
    CommandScheduler.getInstance().registerSubsystem(this);

    hopperMotor.configFactoryDefault();
    hopperMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    hopperMotor.configOpenloopRamp(Constants.hopper.RAMP_RATE);
    hopperMotor.setInverted(true);

    shuffle.startPeriodic(0.2);

    MusicPlayer.getInstance().addMotor(hopperMotor);
  }

  @Override
  public void periodic() {}

  private void updateShuffle() {
    SmartDashboard.putBoolean("Chn. 0", breaker1.get());
    SmartDashboard.putBoolean("Chn. 1", breaker2.get());
    SmartDashboard.putBoolean("Chn. 2", breaker3.get());
    SmartDashboard.putBoolean("Chn. 3", breaker4.get());
  }

  public void set(double power) {
    hopperMotor.set(power);
  }

  public boolean getTopBeam() {
    return !(breaker1.get() && breaker2.get());
  }

  public double getTemperature() {
    return hopperMotor.getTemperature();
  }
}

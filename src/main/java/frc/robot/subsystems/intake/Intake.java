// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.DoubleSolenoid;
import frc.robot.utils.motors.CSPMotor;

public class Intake extends SubsystemBase {
  private static Intake intake;

  public static synchronized Intake getInstance () {
    if (intake == null) intake= new Intake();
    return intake;
  }

  private CSPMotor intakeMotor = Constants.devices.intakeMotor;
  private DoubleSolenoid piston = Constants.devices.intakePiston;

  Notifier shuffle = new Notifier(() -> updateSuffleboard());

  public Intake(){
    CommandScheduler.getInstance().registerSubsystem(this);
    intakeMotor.reset();
    intakeMotor.setRamp(0.125);

    shuffle.startPeriodic(2.0);
    intakeMotor.setInverted(true);

    raise(true);
  }

  @Override
  public void periodic() {
  }

  public void updateSuffleboard() {
    SmartDashboard.putNumber("Intake Temperature", getTemperature());
  }

  public void set(double percentage){
    intakeMotor.set(percentage);
  }

  public void setVoltage(double voltage) {
    set(voltage / RobotController.getBatteryVoltage());
  }

  public double getTemperature() {
    return intakeMotor.getTemperature();
  }

  public void raise(boolean engaged) {
    piston.set(engaged);
  }

  public boolean isRaised() {
    return piston.get();
  } 
}

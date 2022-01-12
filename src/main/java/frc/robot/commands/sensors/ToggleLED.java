// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sensors;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.sensors.Limelight.LedMode;
import frc.robot.subsystems.sensors.Sensors;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToggleLED extends InstantCommand {

  Sensors sensors = Sensors.getInstance();

  public ToggleLED() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LedMode current;

    try {
      current = sensors.getLEDMode();
    } catch (Exception e) {
      current = LedMode.OFF;

      e.printStackTrace();
      System.out.println(e.getMessage());
      DriverStation.reportError(e.getMessage(), true);
    }

    switch (current) {
      case DEFAULT:
        sensors.setLEDMode(LedMode.ON);
      case OFF:
        sensors.setLEDMode(LedMode.ON);
      case BLINK:
        sensors.setLEDMode(LedMode.OFF);
      case ON:
        sensors.setLEDMode(LedMode.OFF);
    }
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensors;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

/** Add your docs here. */
public class ColorSensor {
    private ColorSensorV3 colorSensor = Constants.Sensors.Devices.SENSOR;
    private ColorMatch matchColor = new ColorMatch();

    private Color detected;
    private ColorMatchResult match;
    
    public ColorSensor() {
        refresh();
    }

    public void refresh() {
        detected = colorSensor.getColor();
        match = matchColor.matchClosestColor(detected);
    }

    public String getColor() {
        if (match.color == Constants.Sensors.BLUE && match.confidence > 0.9) return "Blue";
        else if (match.color == Constants.Sensors.RED && match.confidence > 0.9) return "Red";
        else return "None";
    }
}

package frc.robot.subsystems.sensors;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase{
  private ColorSensorV3 colorSensor;
  private ColorMatch matchColor;

  private Color blue = new Color(0.1619, 0.3984, 0.4399);
  private Color red = new Color(0.48, 0.36, 0.16);

  private Color detectedColor;
  private ColorMatchResult match;


  public ColorSensor(I2C.Port i2cPort) {
    colorSensor = new ColorSensorV3(i2cPort);
    matchColor = new ColorMatch();

    matchColor.addColorMatch(red);
    matchColor.addColorMatch(blue);
  }

  /**
   * Blue is 1, Red is -1, and neither is 0.
   */
  public int getColor() {
    if (match.color == red && match.confidence > 0.9) return -1;
    else if (match.color == blue && match.confidence > 0.9) return 1;
    else return 0;
  }

  @Override
  public void periodic() {
    detectedColor = colorSensor.getColor();
    match = matchColor.matchClosestColor(detectedColor);
    SmartDashboard.putNumber("Color Sensed", getColor());
  }
}
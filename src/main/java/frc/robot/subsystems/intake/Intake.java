// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private static Intake intake;

  public static synchronized Intake getInstance () {
    if (intake == null) intake= new Intake();
    return intake;
  }

  private CANSparkMax testMotor = new CANSparkMax(24, MotorType.kBrushless);
  //private WPI_TalonFX intakeMotor = new WPI_TalonFX(14);

 //lets the intake power be set on smart dashboard for training purposes. 
  public Intake() {
    SmartDashboard.putNumber("Test Motor Power", 0);

  }

   public void set(double power) {
    testMotor.set(power);
    //intakeMotor.set(power);
  }




// shows intake motor temp on smart dashboard
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Temperature", testMotor.getMotorTemperature());
    //SmartDashboard.putNumber("Intake Motor Temp", intakeMotor.getMotorTemperature());
    
  }
}

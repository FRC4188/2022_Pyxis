// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.sensors.Sensors;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;

public class AutoMagazine extends CommandBase {

  private Hopper hopper = Hopper.getInstance();
  private Shooter shooter = Shooter.getInstance();
  private Turret turret = Turret.getInstance();
  private boolean cont;

  /** Creates a new AutoMagazine. */
  public AutoMagazine(boolean cont) {
    addRequirements(hopper);

    this.cont = cont;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean ready =
        Math.abs(shooter.getVelocity() - Sensors.getInstance().getFormulaRPM()) < 200.0
            && turret.isAimed();

    System.out.println(String.valueOf(ready));

    if (ready) {
      hopper.set(1.0);
    } else if (hopper.getTopBeam()) {
      hopper.set(0.2);
    } else {
      hopper.set(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !cont;
  }
}

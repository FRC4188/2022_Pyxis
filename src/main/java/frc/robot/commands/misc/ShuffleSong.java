// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.misc;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.MusicPlayer;

public class ShuffleSong extends CommandBase {

  /** Creates a new PlaySong. */
  public ShuffleSong() {
    addRequirements(
        Swerve.getInstance(), Shooter.getInstance(), Hopper.getInstance(), Intake.getInstace());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    MusicPlayer.getInstance().play();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return MusicPlayer.getInstance().isPlaying();
  }
}

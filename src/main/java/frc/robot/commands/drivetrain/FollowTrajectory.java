// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;

/** Add your docs here. */
public class FollowTrajectory extends SwerveControllerCommand {
    private static Drivetrain drivetrain = Drivetrain.getInstance();

    public FollowTrajectory(Trajectory trajectory) {
        super(trajectory, drivetrain::getPose, drivetrain.getKinematics(), Constants.Drivetrain.Controllers.X_CONTROLLER, 
            Constants.Drivetrain.Controllers.Y_CONTROLLER, Constants.Drivetrain.Controllers.ROT_CONTROLLER, drivetrain::setModuleStates, drivetrain);

         Constants.Drivetrain.Controllers.ROT_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
    }




}

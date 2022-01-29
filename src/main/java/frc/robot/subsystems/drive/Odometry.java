package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.utils.math.Integral;

public class Odometry {

    private Integral xPos = null;
    private Integral yPos = null;
    private Rotation2d heading;
    
    public Odometry(Pose2d pose) {
        xPos = new Integral(pose.getX());
        yPos = new Integral(pose.getY());
    }

    public void update(ChassisSpeeds speeds, Rotation2d gyro) {
        double direction = Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond) + gyro.getRadians();
        double speed = Math.hypot(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond);

        xPos.sample(speed * Math.cos(direction));
        yPos.sample(speed * Math.sin(direction));
        heading = gyro;
    }

    public Pose2d getPose() {
        return new Pose2d(xPos.getValue(), yPos.getValue(), heading);
    }

    public void setPose(Pose2d pose) {
        xPos = new Integral(pose.getX());
        yPos = new Integral(pose.getY());
        heading = pose.getRotation();
    }
}

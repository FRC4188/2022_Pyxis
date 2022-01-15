package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.math.Vector;

public class Kinematics {

    Translation2d[] positions;

    public Kinematics(Translation2d... positions) {
        this.positions = positions;
    }

    public ChassisSpeeds forward(SwerveModuleState[] states) {
        
        double totalX = 0.0;
        double totalY = 0.0;

        for (int i = 0; i < states.length; i++) {
            states[i] = new SwerveModuleState(states[i].speedMetersPerSecond, new Rotation2d(states[i].angle.getRadians() - Math.PI / 2.0));
        }

        for (SwerveModuleState module : states) {
            totalX += (Math.sin(module.angle.getRadians()) * module.speedMetersPerSecond) / states.length;
            totalY += (Math.cos(module.angle.getRadians()) * module.speedMetersPerSecond)  / states.length;
        }

        double totalR = 0.0;

        for (int i = 0; i < states.length; i++) {
            double angle = states[i].angle.getRadians() + Math.atan2(positions[i].getY(), positions[i].getX());//(Math.PI / 4.0 + moduleStates[i].angle + positions[i].a) % (2.0 * Math.PI) - Math.PI;
            totalR += Math.sin(-angle) * states[i].speedMetersPerSecond / Math.hypot(positions[i].getY(), positions[i].getX());
        }

        totalR /= states.length;

        return new ChassisSpeeds(
            Math.round(-totalX * 10e7) / 10e7,
            Math.round(totalY * 10e7) / 10e7,
            Math.round(totalR * 10e7) / 10e7
        );
    }

    public SwerveModuleState[] inverse(ChassisSpeeds speeds) {
        SwerveModuleState[] result = new SwerveModuleState[positions.length];

        for (int i = 0; i < positions.length; i++) {
            if (speeds.vxMetersPerSecond == 0.0 && speeds.vyMetersPerSecond == 0.0 && speeds.omegaRadiansPerSecond == 0.0) result[i] = new SwerveModuleState(0.0, new Rotation2d(Math.PI));
            else {
                Vector rot = new Vector(Math.atan2(positions[i].getX(), -positions[i].getY()) + Math.PI, Math.hypot(positions[i].getY(), positions[i].getX()) * speeds.omegaRadiansPerSecond);
                //System.out.println(rot.toString());
                Vector trans = new Vector(Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond) - Math.PI / 2.0, Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));

                rot.addVector(trans);

                result[i] = new SwerveModuleState(rot.magnitude, new Rotation2d(rot.angle + Math.PI / 2.0));
            }
        }

        return result;
    }
}

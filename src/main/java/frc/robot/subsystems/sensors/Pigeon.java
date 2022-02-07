package frc.robot.subsystems.sensors;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.math.Derivative;

public class Pigeon extends PigeonIMU{

    Derivative rate = new Derivative(0.0);

    public Pigeon(int canID) {
        super(canID);

        super.configFactoryDefault();
        super.setFusedHeading(0.0);
        super.setCompassAngle(0.0);
    }

    public Rotation2d get() {
        return Rotation2d.fromDegrees((super.getFusedHeading() + 180.0) % 360.0 - 180.0);
    }

    public Rotation2d getCompass() {
        return Rotation2d.fromDegrees((super.getCompassHeading() + 180.0) % 360.0 - 180.0);
    }

    public double getOmegaRadians() {
        return rate.getRate(Math.toRadians(super.getFusedHeading()));
    }

    public void resetAngle(double angle) {
        setFusedHeading(0.0);
    }
}

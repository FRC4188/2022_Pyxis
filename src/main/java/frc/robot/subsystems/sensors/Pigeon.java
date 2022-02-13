package frc.robot.subsystems.sensors;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.math.Derivative;

public class Pigeon extends Pigeon2{

    Derivative rate = new Derivative(0.0);

    public Pigeon(int canID) {
        super(canID);

        super.configFactoryDefault();
        set(0.0);
    }

    public Rotation2d get() {
        return Rotation2d.fromDegrees((super.getYaw() + 180.0) % 360.0 - 180.0);
    }

    public Rotation2d getCompass() {
        return Rotation2d.fromDegrees((super.getCompassHeading() + 180.0) % 360.0 - 180.0);
    }

    public double getOmegaRadians() {
        return rate.getRate(Math.toRadians(super.getYaw()));
    }

    public void set(double angle) {
        super.setYaw(angle);
    }
}

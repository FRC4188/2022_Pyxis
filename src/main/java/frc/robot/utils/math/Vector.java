package frc.robot.utils.math;

public class Vector {
    
    public double x;
    public double y;
    public double angle;
    public double magnitude;

    public Vector(double angle, double magnitude) {
        x = Math.sin(angle) * magnitude;
        y = Math.cos(angle) * magnitude;
        this.angle = angle;
        this.magnitude = magnitude;
    }

    @Override
    public String toString() {
        return "Angle: " + Math.toDegrees(angle) + "; Magnitude: " + magnitude;
    }

    public void addVector(Vector vec) {
        x += vec.x;
        y += vec.y;

        angle = Math.atan2(x, y);
        magnitude = Math.hypot(x, y);
    }
}

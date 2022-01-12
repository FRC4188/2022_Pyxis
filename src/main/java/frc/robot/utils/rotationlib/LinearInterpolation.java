package frc.robot.utils.rotationlib;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;
import java.util.List;

public class LinearInterpolation {
  ArrayList<Double> rotations;
  ArrayList<Double> times;

  public LinearInterpolation(List<Rotation> rotations, Rotation2d start) {
    times.add(0.0);
    this.rotations.add(start.getDegrees());
    for (Rotation rotation : rotations) {
      this.rotations.add(rotation.getRotation().getDegrees());
      times.add(rotation.getTime());
    }
  }

  public Rotation2d sample(double time) {
    int upperIndex = 0;
    int lowerIndex = 0;

    for (int i = 0; i < times.size(); i++) {
      if (times.get(i) >= time) {
        upperIndex = i;
        lowerIndex = i - 1;
        break;
      }
    }

    if (upperIndex == 0 && lowerIndex == 0) {
      return Rotation2d.fromDegrees(rotations.get(rotations.size() - 1));
    } else {
      double rotation =
          ((rotations.get(upperIndex) - rotations.get(lowerIndex))
                      / (times.get(upperIndex) - times.get(lowerIndex)))
                  * time
              - (((rotations.get(upperIndex) - rotations.get(lowerIndex))
                          / (times.get(upperIndex) - times.get(lowerIndex)))
                      * times.get(lowerIndex)
                  - rotations.get(lowerIndex));

      return Rotation2d.fromDegrees(rotation);
    }
  }
}

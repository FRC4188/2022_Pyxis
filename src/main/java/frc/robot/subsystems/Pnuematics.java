package frc.robot.subsystems;

import frc.robot.utils.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pnuematics extends SubsystemBase{
    private static Pnuematics instance;

  public static synchronized Pnuematics getInstance() {
    if (instance == null) instance = new Pnuematics();
    return instance;
  }

  private DoubleSolenoid piston = new DoubleSolenoid(0, 1);

  public Pnuematics() {
    
  }

  @Override
    public void periodic() {
    }
  public void raise(boolean engaged) {
      piston.set(engaged);
  }
}

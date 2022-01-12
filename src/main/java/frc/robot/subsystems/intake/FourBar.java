package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class FourBar {

  private DoubleSolenoid masterSolenoid;

  public FourBar(int masterID) {
    masterSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);

    masterSolenoid.set(Value.kOff);
  }

  public void setRaised(boolean raised) {
    Value state = (raised) ? Value.kForward : Value.kReverse;

    masterSolenoid.set(state);
  }

  public void relax() {
    masterSolenoid.set(Value.kOff);;
  }

  public boolean getRaised() {
    return (masterSolenoid.get() == Value.kReverse) ? true : false;
  }
}

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.utils.DoubleSolenoid;

public class PassiveHook {

    private DoubleSolenoid piston = new DoubleSolenoid(Constants.climber.SOLENOID_A_ID, Constants.climber.SOLENOID_B_ID);

    private DigitalInput leftIn = new DigitalInput(Constants.climber.LEFT_IN);
    private DigitalInput leftOut = new DigitalInput(Constants.climber.LEFT_OUT);
    private DigitalInput rightIn = new DigitalInput(Constants.climber.RIGHT_IN);
    private DigitalInput rightOut = new DigitalInput(Constants.climber.RIGHT_OUT);

    protected PassiveHook() {
        setPosition(false);
    }

    public void setPosition(boolean output) {
        piston.set(output);
    }

    public boolean getSet() {
        return piston.get();
    }

    public void off() {
        piston.off();
    }

    public boolean getLeftIn() {
        return leftIn.get();
    }
    public boolean getLeftOut() {
        return leftOut.get();
    }
    public boolean getRightIn() {
        return rightIn.get();
    }
    public boolean getRightOut() {
        return rightOut.get();
    }
}

package frc.robot.subsystems.climber;

import frc.robot.Constants;
import frc.robot.utils.DoubleSolenoid;

public class PassiveHook {

    private DoubleSolenoid piston = new DoubleSolenoid(Constants.climber.SOLENOID_A_ID, Constants.climber.SOLENOID_B_ID);

    protected PassiveHook() {
        
    }

    public void setPosition(boolean output) {
        piston.set(output);
    }

    public boolean getPosition() {
        return piston.get();
    }

    public void off() {
        piston.off();
    }
}

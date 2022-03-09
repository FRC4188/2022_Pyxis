package frc.robot.subsystems.climber;

import frc.robot.Constants;
import frc.robot.utils.DoubleSolenoid;

public class PassiveHook {

    private DoubleSolenoid piston = Constants.devices.climberPiston;

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
}

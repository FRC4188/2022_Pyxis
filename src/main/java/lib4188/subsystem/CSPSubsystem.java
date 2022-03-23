package lib4188.subsystem;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib4188.data.Data.Key;

public abstract class CSPSubsystem extends SubsystemBase {

    private Notifier dashThread = new Notifier(() -> updateDashboard());
    private Notifier dataThread = new Notifier(() -> updateData());

    public CSPSubsystem() {
        CommandScheduler.getInstance().registerSubsystem(this);

        dashThread.startPeriodic(0.1);
        dataThread.startPeriodic(0.02);

        startup();
    }
    
    public void startup() {}
    public void updateDashboard() {}
    public void updateData() {}
}

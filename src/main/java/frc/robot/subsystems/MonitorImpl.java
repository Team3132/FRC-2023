package frc.robot.subsystems;



import com.sun.management.OperatingSystemMXBean;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.Monitor;
import frc.robot.lib.Subsystem;
import frc.robot.lib.chart.Chart;
import java.lang.management.ManagementFactory;

/**
 * Records the RoboRIO's CPU usage and memory usage to charts and SmartDashboard. Could be useful
 * for debugging or checking if CPU/RAM usage is unusually high.
 */
public class MonitorImpl extends Subsystem implements Monitor {
    OperatingSystemMXBean osBean;
    double systemMemory;

    public MonitorImpl() {
        super("Monitor");
        osBean = ManagementFactory.getPlatformMXBean(OperatingSystemMXBean.class);
        systemMemory = osBean.getTotalMemorySize() / 1024 / 1024; // Convert bytes to MB
        Chart.register(() -> getCpuUsage(), "%s/CPU", name);
        Chart.register(() -> getMemoryUsed(), "%s/Memory Used Ratio", name);
    }

    private double getCpuUsage() {
        return osBean.getCpuLoad();
    }

    private double getMemoryUsed() {
        return (systemMemory - osBean.getFreeMemorySize() / 1024 / 1024) / systemMemory;
    }

    /**
     * Update the operator console with the status of the RoboRIO.
     */
    @Override
    public void updateDashboard() {
        SmartDashboard.putNumber("CPU", getCpuUsage());
        SmartDashboard.putNumber("Memory Used Ratio", getMemoryUsed());
    }
}

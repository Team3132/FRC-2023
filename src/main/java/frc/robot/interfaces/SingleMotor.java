package frc.robot.interfaces;

import org.strongback.Executable;

public interface SingleMotor extends Subsystem, Executable, DashboardUpdater {
    public void setDutyCycle(double dutyCycle);

    public double getDutyCycle();
}

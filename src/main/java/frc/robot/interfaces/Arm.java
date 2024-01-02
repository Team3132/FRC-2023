package frc.robot.interfaces;

import org.strongback.Executable;

/**
 * Arm with an intake on the end, mounted to a pivot actuated by 2 Falcons. Uses external
 * FeedForward and ProfiledPid controllers
 */
public interface Arm extends Subsystem, Executable, DashboardUpdater {

    /**
     * Returns if the pivot and telescope are in position
     * 
     * @return boolean isArmInPosition
     */
    public boolean isInPosition();

    /**
     * Sets the target angle of the arm in degrees
     * 
     * @param degrees
     */
    public void setTargetAngle(double degrees);

    /**
     * Gets the target angle of the arm in degrees
     * 
     * @return degrees
     */
    public double getTargetAngle();

    /**
     * Gets the current angle of the arm in degrees
     * 
     * @return angle in degrees
     */
    public double getAngle();

    /**
     * Sets the target position of the telescope in metres
     * 
     * @param metres
     */
    public void setExtension(double metres);

    /**
     * Gets the current position of the telescope in metres
     * 
     * @return metres
     */
    public double getExtension();

    /**
     * Triggers a calibration
     */
    public void forceCalibration();

}

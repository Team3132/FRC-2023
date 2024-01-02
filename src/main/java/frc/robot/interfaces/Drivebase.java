package frc.robot.interfaces;


import org.strongback.Executable;
import frc.robot.drive.modes.Mode;
import frc.robot.drive.modes.Mode.Parameters;
import frc.robot.drive.modes.Mode.Type;

/**
 * The Drivebase subsystem is responsible for dealing with the drivebase.
 * It will call the location subsystem when things on the drivebase change, and it
 * requests information from the DriveControl to tell it how to move.
 * 
 * The Drivebase is passed the motors and other devices it uses and implements the
 * control algorithms needed to co-ordinate actions on these devices.
 */
public abstract interface Drivebase
        extends DriveTelemetry, Executable, Subsystem, DashboardUpdater, LogHelper {
    /**
     * The values to give to the motors on each side of the robot.
     */
    public class DriveMotion {
        public double left;
        public double right;

        public DriveMotion(double left, double right) {
            this.left = left;
            this.right = right;
        }

        @Override
        public String toString() {
            return "Left: " + left + ", Right: " + right;
        }

        public boolean equals(Object o) {
            if (o == this) {
                return true;
            }
            if (!(o instanceof DriveMotion)) {
                return false;
            }
            DriveMotion m = (DriveMotion) o;
            return m.left == left && m.right == right;
        }

        public int hashCode() {
            return (int) (1000 * left + right);
        }
    }

    /**
     * Tell the drivebase what action/drive mode to operate in.
     * 
     * @param parameters
     */
    public void setMode(Parameters parameters);

    public default void setArcadeDrive() {
        setMode(Mode.getArcade());
    }

    /**
     * Get the action that was requested of the drivebase.
     * 
     * @return
     */
    public Parameters getParameters();

    /**
     * Returns false if the drivebase has more to do.
     * Only Trajectory drive can return false in case it has
     * more driving to do.
     */
    public boolean hasFinished();

    /**
     * Register with the drivebase a way to drive the requested mode by using the supplied mode.
     */
    public void register(Type type, Mode mode);
}

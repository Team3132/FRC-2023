package frc.robot.drive.modes;



import frc.robot.interfaces.Drivebase.DriveMotion;
import frc.robot.interfaces.LogHelper;
import frc.robot.lib.MathUtil;
import frc.robot.lib.TrajectoryLib;
import frc.robot.lib.log.Log;
import java.io.IOException;
import java.util.List;
import org.strongback.components.Motor.ControlMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;

/*
 * This interface class defines the interface to the drive controls.
 * 
 * Each drive mode operates independently.
 */
public abstract class Mode implements LogHelper {
    protected String name;
    protected ControlMode controlMode;

    public static enum Type {
        CONSTANT_POWER, // Set a constant power to drive wheels.
        CONSTANT_SPEED, // Set a constant speed to drive wheels.
        ARCADE_DUTY_CYCLE, // Normal arcade drive.
        CHEESY, // Cheesy drive using the drivers joysticks.
        TRAJECTORY, // Drive through waypoints.
        // TODO: Remove the old vision routines and the vision subsystem.
        VISION_DRIVE, // Drive using the camera to the vision goal.
        VISION_AIM, // Use the camera to turn the robot towards the goal for autonomous.
        VISION_ASSIST, // Use the camera to help the driver towards the goal.
        TAPE_ASSIST, // Driver has speed control and tape subsystem has turn control.
        TURN_TO_BEARING, // Turn to specified bearing. Normally the robot starts pointing "North".
        POSITION_PID_ARCADE, // Use the joystick to drive with positional PID.
        ARCADE_VELOCITY, // Normal arcade drive.
        BALANCE, DDRPAD_DRIVE; // Dance Dance Revolution Pad drive.

        static public Type get(String name) {
            try {
                return Type.valueOf(name);
            } catch (Exception e) {
                Log.error("Drivebase",
                        "Invalid drive routine type: %s. Using ARCADE_DUTY_CYCLE", name);
                return Type.ARCADE_DUTY_CYCLE;
            }
        }
    }

    protected Mode(String name, ControlMode controlMode) {
        this.name = name;
        this.controlMode = controlMode;
    }

    /*
     * Whether to drive both fieldConfig, or just the left or right side for our Arcade to Tank
     * conversion.
     */
    public enum DriveSide {
        BOTH, LEFT, RIGHT
    };

    /**
     * Configuration parameters for this routine.
     */
    public static class Parameters {
        public Parameters(Type type) {
            this.type = type;
        } // Disable.

        public Type type = Type.ARCADE_DUTY_CYCLE;

        // Waypoint parameters.
        public Trajectory trajectory;
        public boolean relative = true;

        // Constant drive parameters
        public double value = 0;
        public boolean forward = false;

        @Override
        public boolean equals(Object obj) {
            if (obj == null)
                return false;
            if (!(obj instanceof Parameters))
                return false;
            if (obj == this)
                return true;
            Parameters other = (Parameters) obj;
            return type == other.type && value == other.value && trajectory == other.trajectory
                    && relative == other.relative;
        }

        @Override
        public String toString() {
            if (type == Type.CONSTANT_POWER) {
                return String.format("constant power %.1f", value);
            }
            if (type == Type.CONSTANT_SPEED) {
                return String.format("constant speed %.1f", value);
            }
            if (type == Type.TURN_TO_BEARING) {
                return String.format("turn to angle %.1f", value);
            }
            return String.format("routine=%s", type.toString().toLowerCase());
        }
    }

    public static Parameters getConstantPower(double power) {
        Parameters p =
                new Parameters(Type.CONSTANT_POWER);
        p.value = power;
        return p;
    }

    public static Parameters getConstantSpeed(double speed) {
        Parameters p =
                new Parameters(Type.CONSTANT_SPEED);
        p.value = speed;
        return p;
    }

    public static Parameters getArcade() {
        Parameters p =
                new Parameters(Type.ARCADE_DUTY_CYCLE);
        return p;
    }

    public static Parameters getDriveWaypoints(String filename) throws IOException {
        Parameters p =
                new Parameters(Type.TRAJECTORY);
        p.trajectory = TrajectoryLib.getTrajectory(filename);
        return p;
    }

    public static Parameters getBalance(boolean forward) {
        Parameters p =
                new Parameters(Type.BALANCE);
        p.forward = forward;
        return p;
    }

    public static Parameters getDriveWaypoints(Pose2d start,
            List<Translation2d> interiorWaypoints,
            Pose2d end, boolean forward, boolean relative) {
        Parameters p =
                new Parameters(Type.TRAJECTORY);
        p.trajectory = TrajectoryLib.generateTrajectory(start, interiorWaypoints, end, forward,
                relative);
        p.relative = relative;
        return p;
    }

    public static Parameters turnToAngle(double angle) {
        Parameters p =
                new Parameters(Type.TURN_TO_BEARING);
        p.value = angle;
        return p;
    }

    public static Parameters positionPIDArcade() {
        return new Parameters(Type.POSITION_PID_ARCADE);
    }

    /**
     * This drive routine was requested by an action.
     * Get any necessary details from it (eg waypoints)
     * 
     * @param parameters
     */
    public void reset(Parameters parameters) {}

    /**
     * DriveMotion determines the power that should be applied to the left and right
     * hand fieldConfig of the robot by the drivebase.
     * 
     * @param leftSpeed The current speed of the robot on the left side.
     * @param rightSpeed The current speed of the robot on the right side.
     * @return The power to apply to each side of the robot by the drivebase.
     */
    public abstract DriveMotion getMotion(double leftSpeed, double rightSpeed);

    /**
     * Returns true if there is nothing more to do.
     * Used by the controller to know if it needs to keep waiting
     * Most routines will return true.
     * 
     * @return if no more time is needed to finish the current driving.
     */
    public boolean hasFinished() {
        return true;
    }

    /**
     * Return the name of the Drive Control
     * 
     * @return name of the drive control
     */
    public String getName() {
        return name;
    }

    /**
     * Return the name of the Drive Control
     * 
     * @return name of the drive control
     */
    public ControlMode getControlMode() {
        return controlMode;
    }

    /**
     * Activate this driveControl. Perform any initialisation needed with the assumption
     * that the robot is currently in the correct position
     */
    public void enable() {

    }

    /**
     * Prepare the drive control for deactivation. Stop all independent tasks and safe all controls.
     * Deactivate can be called before activate.
     */
    public void disable() {

    }

    public double limit(double value) {
        if (value < -1.0)
            value = -1.0;
        if (value > 1.0)
            value = 1.0;
        return value;
    }

    public DriveMotion arcadeToTank(double moveValue, double turnValue, double scale) {
        return arcadeToTank(moveValue, turnValue, scale, DriveSide.BOTH);
    }

    public DriveMotion arcadeToTank(double moveValue, double turnValue, double scale,
            DriveSide driveSide) {
        // double im = moveValue;
        // double it = turnValue;
        moveValue = limit(moveValue);
        turnValue = limit(turnValue);
        double leftMotorSpeed = 0;
        double rightMotorSpeed = 0;

        if (moveValue > 0.0) {
            if (turnValue > 0.0) {
                leftMotorSpeed = moveValue - turnValue;
                rightMotorSpeed = Math.max(moveValue, turnValue);
            } else {
                leftMotorSpeed = Math.max(moveValue, -turnValue);
                rightMotorSpeed = moveValue + turnValue;
            }
        } else {
            if (turnValue > 0.0) {
                leftMotorSpeed = -Math.max(-moveValue, turnValue);
                rightMotorSpeed = moveValue + turnValue;
            } else {
                leftMotorSpeed = moveValue - turnValue;
                rightMotorSpeed = -Math.max(-moveValue, -turnValue);
            }
        }
        /*
         * Adjust here for left and right only changes if necessary.
         */
        switch (driveSide) {
            default:
            case BOTH:
                // all is good. Do nothing
                break;
            case LEFT:
                leftMotorSpeed = leftMotorSpeed - rightMotorSpeed;
                rightMotorSpeed = 0.0;
                break;
            case RIGHT:
                leftMotorSpeed = 0.0;
                rightMotorSpeed = rightMotorSpeed - leftMotorSpeed;
                break;
        }
        leftMotorSpeed = scale * MathUtil.clamp(leftMotorSpeed, -1.0, 1.0);
        rightMotorSpeed = scale * MathUtil.clamp(rightMotorSpeed, -1.0, 1.0);
        // System.out.printf("A2T(%f, %f) -> %f,%f\n", im, it, leftMotorSpeed, rightMotorSpeed);
        return new DriveMotion(leftMotorSpeed, rightMotorSpeed);
    }
}

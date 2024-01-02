package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.drive.modes.ConstantDrive;
import frc.robot.drive.modes.Mode;
import frc.robot.drive.modes.Mode.Parameters;
import frc.robot.drive.modes.Mode.Type;
import frc.robot.interfaces.Drivebase;
import frc.robot.lib.Subsystem;
import frc.robot.lib.chart.Chart;
import java.util.Map;
import java.util.TreeMap;
import org.strongback.components.Motor;
import org.strongback.components.Motor.ControlMode;

/**
 * Subsystem responsible for the drivetrain
 *
 * Normally there are multiple motors per side, but these have been set to
 * follow one motor per side and these are passed to the drivebase.
 *
 * It periodically queries the joysticks (or the auto routine) for the
 * speed/power for each side of the drivebase.
 */
public class DrivebaseImpl extends Subsystem implements Drivebase {
    private Parameters parameters =
            Mode.getConstantPower(0);
    private Mode mode = null;
    private ControlMode controlMode = ControlMode.DutyCycle; // The mode the talon should be in.
    private final Motor left;
    private final Motor right;
    private DriveMotion currentMotion;

    public DrivebaseImpl(Motor left, Motor right) {
        super("Drive");
        this.left = left;
        this.right = right;

        currentMotion = new DriveMotion(0, 0);
        mode = new ConstantDrive("Constant Drive", ControlMode.DutyCycle);
        disable(); // disable until we are ready to use it.
        Chart.register(() -> currentMotion.left, "%s/setpoint/Left", name);
        Chart.register(() -> currentMotion.right, "%s/setpoint/Right", name);
        Chart.register(() -> left.getPosition(), "%s/position/Left", name);
        Chart.register(() -> right.getPosition(), "%s/position/Right", name);
        Chart.register(() -> left.getSpeed(), "%s/speed/Left", name);
        Chart.register(() -> right.getSpeed(), "%s/speed/Right", name);
        Chart.register(() -> left.getOutputVoltage(), "%s/outputVoltage/Left", name);
        Chart.register(() -> right.getOutputVoltage(), "%s/outputVoltage/Right", name);
        Chart.register(() -> left.getOutputPercent(), "%s/outputPercentage/Left", name);
        Chart.register(() -> right.getOutputPercent(), "%s/outputPercentage/Right", name);
        Chart.register(() -> left.getOutputCurrent(), "%s/outputCurrent/Left", name);
        Chart.register(() -> right.getOutputCurrent(), "%s/outputCurrent/Right", name);
    }

    @Override
    public void setMode(Parameters parameters) {
        if (this.parameters != null && parameters.equals(this.parameters)) {
            // debug("Parameters are identical not setting these");
            return;
        }
        // Drive mode has changed.
        this.parameters = parameters; // Remember it for next time.
        // Find a mode to handle it
        Mode newMode = driveModes.getOrDefault(parameters.type, null);
        if (newMode == null) {
            error("Bad drive mode %s", parameters.type);
            return;
        }
        // Tell the drive mode to change what it is doing.
        newMode.reset(parameters);
        debug("Switching to %s drive mode using ControlMode %s", newMode.getName(),
                newMode.getControlMode());
        if (mode != null)
            mode.disable();
        newMode.enable();
        mode = newMode;
        controlMode = newMode.getControlMode();
    }

    @Override
    public Parameters getParameters() {
        return parameters;
    }

    @Override
    synchronized public void update() {
        // Query the drive mode for the desired wheel speed/power.
        if (mode == null)
            return; // No drive mode set yet.
        // Ask for the power to supply to each side. Pass in the current wheel speeds.
        DriveMotion motion = mode.getMotion(left.getSpeed(), right.getSpeed());
        // Logger.debug("drive subsystem motion = %.1f, %.1f", motion.left,
        // motion.right);
        if (motion.equals(currentMotion)) {
            return; // No change.
        }
        // The TalonSRX doesn't have a watchdog (unlike the WPI_ version), so no need to
        // updated it often.
        currentMotion = motion; // Save it for logging.
        left.set(controlMode, motion.left);
        right.set(controlMode, motion.right);
    }

    @Override
    public void enable() {

        super.enable();

        if (mode != null)
            mode.enable();
    }

    public void disable() {
        super.disable();
        if (mode != null)
            mode.disable();
        left.set(ControlMode.DutyCycle, 0.0);
        right.set(ControlMode.DutyCycle, 0.0);
        currentMotion.left = 0;
        currentMotion.right = 0;
    }

    @Override
    public void updateDashboard() {
        SmartDashboard.putNumber("Left drive motor duty cycle", currentMotion.left);
        SmartDashboard.putNumber("Right drive motor duty cycle", currentMotion.right);
        SmartDashboard.putNumber("Left drive pos", left.getPosition());
        SmartDashboard.putNumber("Right drive pos", right.getPosition());
        SmartDashboard.putString("Drive control", mode.getName());
    }

    /**
     * Will return false if the current drive mode wants to keep control. For
     * example, spline driving will want to keep driving until it's done.
     */
    @Override
    public boolean hasFinished() {
        return mode.hasFinished();
    }

    private Map<Mode.Type, Mode> driveModes =
            new TreeMap<Mode.Type, Mode>();

    @Override
    public void register(Type type, Mode mode) {
        debug("Registered %s drive mode", mode.getName());
        driveModes.put(type, mode);
    }

    @Override
    public void setLeftDistance(double pos) {
        left.resetEncoder(pos);
    }

    @Override
    public void setRightDistance(double pos) {
        right.resetEncoder(pos);
    }

    @Override
    public double getLeftDistance() {
        return left.getPosition();
    }

    @Override
    public double getRightDistance() {
        return right.getPosition();
    }

    @Override
    public double getLeftSpeed() {
        return left.getSpeed();
    }

    @Override
    public double getRightSpeed() {
        return right.getSpeed();
    }
}

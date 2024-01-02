package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Config;
import frc.robot.interfaces.LogHelper;
import frc.robot.lib.MathUtil;
import frc.robot.lib.chart.Chart;
import org.strongback.components.Motor;
import org.strongback.components.Motor.ControlMode;

class PivotArm implements LogHelper {

    private enum State {
        INIT(0), // Runs once to begin calibration
        MOVING_REVERSE(1), // Moves arm in reverse until limit switch is triggered
        MOVING_FORWARDS(2), // Moves arm forwards until limit switch is no longer triggered
        RESET_ENCODER(3), // Reset the encoder to calibrate.
        CALIBRATED(4); // Calibration complete, PID loop is running

        public final int id;

        State(int id) {
            this.id = id;
        }
    }

    private final Motor motor;
    private final String name;

    private State state = Config.arm.pivot.doCalibration ? State.INIT : State.RESET_ENCODER;

    private double targetDegrees = Config.arm.pivot.calibrationAngle;
    private boolean firstRun = true;

    // ArmFeedforward and TrapezoidProfile use radians, so we need to convert
    private final ArmFeedforward feedForwardMin =
            new ArmFeedforward(Config.arm.pivot.feedforward.min.kS,
                    Config.arm.pivot.feedforward.min.kG, Config.arm.pivot.feedforward.min.kV,
                    Config.arm.pivot.feedforward.min.kA);
    private final ArmFeedforward feedForwardMax =
            new ArmFeedforward(Config.arm.pivot.feedforward.max.kS,
                    Config.arm.pivot.feedforward.max.kG, Config.arm.pivot.feedforward.max.kV,
                    Config.arm.pivot.feedforward.max.kA);
    private final ProfiledPIDController pid =
            new ProfiledPIDController(Config.arm.pivot.pidf.p, Config.arm.pivot.pidf.i,
                    Config.arm.pivot.pidf.d,
                    new TrapezoidProfile.Constraints(
                            Math.toRadians(Config.arm.pivot.kMaxVelocity),
                            Math.toRadians(Config.arm.pivot.kMaxAcceleration)));

    public PivotArm(String name, Motor motor) {
        this.name = name;
        this.motor = motor;

        Chart.register(() -> getTargetAngle(), "%s/pivot/target", name);
        Chart.register(() -> getTargetAngle() - getAngle(), "%s/pivot/error", name);

        Chart.register(() -> state.id, "%s/pivot/state", name);

        Chart.register(() -> motor.getSupplyCurrent(), "%s/pivot/motor/current", name);
        Chart.register(() -> motor.getOutputVoltage(), "%s/pivot/motor/voltage", name);
        Chart.register(() -> getAngle(), "%s/pivot/motor/position", name);
        Chart.register(() -> motor.getSpeed(), "%s/pivot/motor/velocity", name);
        Chart.register(() -> motor.getOutputPercent(), "%s/pivot/motor/dutyCycle", name);

        Chart.register(() -> motor.isAtForwardLimit(), "%s/pivot/atForwardLimit", name);
        Chart.register(() -> motor.isAtReverseLimit(), "%s/pivot/atReverseLimit", name);

        Chart.register(() -> Math.toDegrees(pid.getSetpoint().position),
                "%s/pivot/pid/position/goal", name);
        Chart.register(() -> Math.toDegrees(pid.getPositionError()),
                "%s/pivot/pid/position/error", name);
        Chart.register(() -> Math.toDegrees(pid.getSetpoint().velocity),
                "%s/pivot/pid/velocity/goal", name);
        Chart.register(() -> Math.toDegrees(pid.getVelocityError()),
                "%s/pivot/pid/velocity/error", name);

        Chart.register(() -> Math.toDegrees(pid.getSetpoint().position),
                "%s/pivot/profile/position", name);
        Chart.register(() -> Math.toDegrees(pid.getSetpoint().velocity),
                "%s/pivot/profile/velocity", name);
    }

    @Override
    public String getName() {
        return name;
    }

    public void enable() {
        if (!isCalibrated() && Config.arm.pivot.doCalibration) {
            debug("enable() setting state to init");
            state = State.INIT;
        } else {
            debug("enable() setting pid to position %.2f", getAngle());
            pid.reset(Math.toRadians(getAngle()));
        }
    }

    public void disable() {
        motor.set(ControlMode.Disabled, 0);
    }

    public void setTargetAngle(double degrees) {
        if (firstRun) {
            firstRun = false;
        } else if (degrees == targetDegrees) {
            return;
        }
        debug("Target angle set to %.2f", degrees);
        targetDegrees = degrees;
        pid.setGoal(Math.toRadians(degrees));
    }

    public double getTargetAngle() {
        return targetDegrees;
    }

    public double getAngle() {
        return motor.getPosition();
    }

    public boolean isInPosition() {
        if (!isCalibrated()) {
            return false;
        }
        return Math
                .abs(motor.getPosition()
                        - targetDegrees) < Config.arm.pivot.angleToleranceDegrees;
    }

    public boolean isCalibrated() {
        return state == State.CALIBRATED;
    }

    private void setDutyCycle(double dutyCycle) {
        motor.set(ControlMode.DutyCycle, dutyCycle);
    }

    /**
     * Calculates the feedforward in volts to apply to the motor input.
     * 
     * Because the arm can be extended, the feedforward input needs to change. This pro-rata's the
     * calculated min and max values based on the arm extension.
     * 
     * @return volts to add to the motor output.
     */
    double calculateFeedForward(double extension) {
        TrapezoidProfile.State state = pid.getSetpoint();
        double min = feedForwardMin.calculate(state.position, state.velocity);
        double max = feedForwardMax.calculate(state.position, state.velocity);
        // Calculate the ratio of arm extension.
        double ratio = (extension - Config.arm.telescope.minExtension)
                / (Config.arm.telescope.maxExtension - Config.arm.telescope.minExtension);
        ratio = MathUtil.clamp(ratio, 0, 1);
        // Mix in the correct part of the ff calculations based on the extension.
        return (1 - ratio) * min + ratio * max;
    }

    public void forceCalibration() {
        state = State.INIT;
        // Keeps arm at calibration angle
        setTargetAngle(Config.arm.pivot.calibrationAngle);
    }

    /*
     * Calibration process
     * 1. Move arm in reverse until limit switch is triggered
     * 2. Move arm forwards just a bit until limit switch is no longer triggered
     * We do this so that the calibration point is more consistent, not just any point where the
     * beam break is triggered
     * 3. Set position of encoders to match the real angle at this point
     * 4. Set state to CALIBRATED and start PID loop
     */
    public void update(double extension) {
        switch (state) {
            case INIT: {
                info("Starting calibration routine, moving arm in reverse");
                setDutyCycle(Config.arm.pivot.calibrationDutyCycle);
                state = State.MOVING_REVERSE;
                break;
            }
            case MOVING_REVERSE: {
                if (motor.isAtForwardLimit()) {
                    info("Limit reached, moving arm forwards");
                    setDutyCycle(-Config.arm.pivot.calibrationDutyCycle);
                    state = State.MOVING_FORWARDS;
                }
                break;
            }
            case MOVING_FORWARDS: {
                if (!motor.isAtForwardLimit()) {
                    info("No longer at limit, stopping arm");
                    state = State.RESET_ENCODER;
                }
                break;
            }
            case RESET_ENCODER: {
                // Set position of encoders, should match target position.
                motor.resetEncoder(Config.arm.pivot.calibrationAngle);
                double radians = Math.toRadians(Config.arm.pivot.calibrationAngle);
                pid.reset(radians);
                state = State.CALIBRATED;
                info("Encoders reset, calibration complete");
                break;
            }
            case CALIBRATED: {
                double rad = Math.toRadians(getAngle());
                double volts = pid.calculate(rad);
                double demand = calculateFeedForward(extension) + volts;
                motor.set(ControlMode.Voltage, demand);
                break;
            }
            default:
                throw new IllegalArgumentException(
                        String.format("Passed argument %s unknown", state));
        }
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("Arm Target Degrees", targetDegrees);
        SmartDashboard.putNumber("Arm Degrees", getAngle());
        SmartDashboard.putNumber("Arm Voltage", motor.getOutputVoltage());
        SmartDashboard.putNumber("Arm Current", motor.getOutputCurrent());
        SmartDashboard.putNumber("Arm Position Error", targetDegrees - getAngle());
        SmartDashboard.putBoolean("Arm Forward Limit", motor.isAtForwardLimit());
        SmartDashboard.putBoolean("Arm Reverse Limit", motor.isAtReverseLimit());
        SmartDashboard.putString("Arm State", state.toString());

    }
}

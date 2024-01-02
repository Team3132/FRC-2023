package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Config;
import frc.robot.interfaces.LogHelper;
import frc.robot.lib.MathUtil;
import frc.robot.lib.chart.Chart;
import org.strongback.components.Motor;
import org.strongback.components.Motor.ControlMode;

/**
 * This class implements the telescoping functionality of the robot's arm.
 * The extension is powered by one motor.
 * It can calculate feedforward based on the extension of the arm to compensate for gravity.
 * This class is used in ArmImpl.java in conjunction with PivotArm.
 */
class TelescopeArm implements LogHelper {

    private final Motor motor;
    private final String name;

    private double targetExtension = Config.arm.telescope.minExtension;

    public TelescopeArm(String name, Motor motor) {
        this.name = name;
        this.motor = motor;

        Chart.register(() -> motor.getSpeed(), "%s/telescope/motor/velocity", name);
        Chart.register(() -> motor.getSupplyCurrent(), "%s/telescope/motor/current", name);
        Chart.register(() -> motor.getOutputVoltage(), "%s/telescope/motor/voltage", name);
        Chart.register(() -> motor.getOutputPercent(), "%s/telescope/motor/dutyCycle", name);

        Chart.register(() -> getExtension(), "%s/telescope/extn/actual", name);
        Chart.register(() -> getTargetExtension(), "%s/telescope/extn/target", name);
        Chart.register(() -> getExtensionError(), "%s/telescope/extn/error", name);
    }

    @Override
    public String getName() {
        return name;
    }

    public void disable() {
        motor.set(ControlMode.Disabled, 0);
    }

    public double getTargetExtension() {
        return targetExtension;
    }

    public double getExtension() {
        return motor.getPosition() - Config.arm.telescope.baseExtension;
    }

    public double getExtensionError() {
        return getExtension() - targetExtension;
    }

    public boolean isInPosition() {
        return Math.abs(getExtensionError()) <= Config.arm.telescope.positionErrorTolerance;
    }

    public synchronized void setExtension(double metres) {
        metres = MathUtil.clamp(metres, Config.arm.telescope.minExtension,
                Config.arm.telescope.maxExtension);
        if (metres == targetExtension) {
            return;
        }
        debug("Arm extension set to %.3f", metres);
        targetExtension = metres;
    }

    /**
     * Based on the angle of the arm, add extra power to the motor to compensate for gravity when
     * extending and retracting the arm.
     * 
     * @return duty cycle in the range of [-1, 1].
     */
    public double calculateFeedForward(double degrees) {
        // 0 and 180 are out flat and 90 is straight up.
        // Want to return kG for 90 degrees and 0 for 0 degrees.
        double ff = Math.sin(Math.toRadians(degrees)) * Config.arm.telescope.feedforward.kG;
        return MathUtil.clamp(ff, -1, 1);
    }

    public void update(boolean isCalibrating, double degrees) {
        double extension = getTargetExtension();
        if (isCalibrating) {
            // Retract arm so it doesn't get in the way of calibration
            extension = 0;
        }
        // Use motion magic to automatically generate a profile based on the distance to travel.
        // Add in a feedforward factor to compensate for gravity when the arm isn't level.
        double feedforwardDutyCycle = calculateFeedForward(degrees);
        motor.set(ControlMode.MotionMagic, extension + Config.arm.telescope.baseExtension,
                feedforwardDutyCycle);
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("Arm Extension", getExtension());
        SmartDashboard.putNumber("Arm Target Extension", getTargetExtension());
        SmartDashboard.putNumber("Arm Extension Error", getExtensionError());
    }
}

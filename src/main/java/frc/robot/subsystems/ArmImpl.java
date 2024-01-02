package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Config;
import frc.robot.interfaces.Arm;
import frc.robot.lib.Subsystem;
import frc.robot.lib.chart.Chart;
import org.strongback.components.Motor;

/**
 * Arm with an intake on the end, mounted to a pivot actuated by 2 Falcons. Uses the Falcon inbuilt
 * encoder to track angle of pivot; uses a string potentiometer to track extension. Uses motion
 * magic in the talonSRX and feedforward based on the angle of the arm to compensate for gravity.
 */
public class ArmImpl extends Subsystem implements Arm {

    private final PivotArm pivot;
    private final TelescopeArm telescope;

    public ArmImpl(Motor pivotMotor, Motor telescopeMotor) {
        super("arm");
        pivot = new PivotArm(name, pivotMotor);
        telescope = new TelescopeArm(name, telescopeMotor);


        Chart.register(() -> telescope.calculateFeedForward(getAngle()), "%s/telescope/feedforward",
                name);

        // Height limit
        Chart.register(() -> calculateCurrentHeight(getAngle(), getExtension()),
                "%s/telescope/height/actual", name);
        Chart.register(() -> Config.arm.telescope.heightLimit, "%s/telescope/height/limit", name);

        Chart.register(() -> pivot.calculateFeedForward(getExtension()),
                "%s/pivot/feedForward/volts", name);

        // Set target here so that if target is changed during calibration, it is not overwritten
        setTargetAngle(Config.arm.pivot.calibrationAngle);
    }

    @Override
    public double getTargetAngle() {
        return pivot.getTargetAngle();
    }

    @Override
    public void setTargetAngle(double degrees) {
        pivot.setTargetAngle(degrees);
    }

    @Override
    public double getAngle() {
        return pivot.getAngle();
    }

    @Override
    public void enable() {
        super.enable();
        pivot.enable();
        setExtension(getExtension());
    }

    @Override
    public void disable() {
        super.disable();
        pivot.disable();
        telescope.disable();
    }

    @Override
    public boolean isInPosition() {
        return pivot.isInPosition() && telescope.isInPosition();
    }

    @Override
    public void update() {
        pivot.update(getExtension());
        telescope.update(!pivot.isCalibrated(), pivot.getAngle());
    }

    @Override
    public double getExtension() {
        return telescope.getExtension();
    }

    // Make the arm wait until the maximum pivot before extending. The alternatives are to let it
    // extend then retract, which is unsafe, or to wait for the armPivot to stop and then extend,
    // which is slow.
    @Override
    public synchronized void setExtension(double metres) {
        telescope.setExtension(metres);
    }

    @Override
    public void forceCalibration() {
        pivot.forceCalibration();
    }

    /**
     * Calculates the current height of the arm in metres.
     * 
     * @param pivotAngle
     * @param telescopeExtension
     * @return height
     */
    public static double calculateCurrentHeight(double angle, double extension) {
        double heightAbovePivot = (extension + Config.arm.telescope.baseLength)
                * Math.sin(Math.toRadians(angle));
        return Config.arm.telescope.pivotJointHeight + heightAbovePivot;
    }

    @Override
    public void updateDashboard() {
        telescope.updateDashboard();
        pivot.updateDashboard();
        SmartDashboard.putNumber("Arm Extension FF", telescope.calculateFeedForward(getAngle()));
    }
}

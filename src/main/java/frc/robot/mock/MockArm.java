package frc.robot.mock;

import frc.robot.interfaces.Arm;
import frc.robot.lib.Subsystem;

public class MockArm extends Subsystem implements Arm {

    private double angle;
    private double extension;

    public MockArm() {
        super("MockArm");
    }

    @Override
    public void setTargetAngle(double pos) {
        angle = pos;
    }

    @Override
    public double getAngle() {
        return angle;
    }

    @Override
    public double getTargetAngle() {
        return angle;
    }

    @Override
    public void setExtension(double metres) {
        extension = metres;

    }

    @Override
    public double getExtension() {
        return extension;
    }

    @Override
    public boolean isInPosition() {
        return true;
    }

    @Override
    public void forceCalibration() {}
}

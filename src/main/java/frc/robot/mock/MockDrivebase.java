package frc.robot.mock;

import frc.robot.drive.modes.Mode;
import frc.robot.drive.modes.Mode.Parameters;
import frc.robot.drive.modes.Mode.Type;
import frc.robot.interfaces.Drivebase;

public class MockDrivebase implements Drivebase {
    private Parameters parameters =
            new Parameters(Type.ARCADE_DUTY_CYCLE);
    String name = "MockDrivebase";;

    public MockDrivebase() {}

    @Override
    public void execute(long timeInMillis) {}

    @Override
    public String getName() {
        return name;
    }

    @Override
    public void enable() {}

    @Override
    public void disable() {}

    @Override
    public boolean isEnabled() {
        return false;
    }

    @Override
    public void cleanup() {}

    @Override
    public void setMode(Parameters parameters) {
        this.parameters = parameters;
    }

    @Override
    public Parameters getParameters() {
        return parameters;
    }

    @Override
    public boolean hasFinished() {
        return true;
    }

    @Override
    public void register(Type type, Mode mode) {}

    @Override
    public void setLeftDistance(double pos) {}

    @Override
    public void setRightDistance(double pos) {}

    @Override
    public double getLeftDistance() {
        return 0;
    }

    @Override
    public double getRightDistance() {
        return 0;
    }

    @Override
    public double getLeftSpeed() {
        return 0;
    }

    @Override
    public double getRightSpeed() {
        return 0;
    }
}

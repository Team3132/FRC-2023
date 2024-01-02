package frc.robot.mock;

import frc.robot.interfaces.SingleMotor;
import frc.robot.lib.Subsystem;

public class MockSingleMotor extends Subsystem implements SingleMotor {

    private double dutyCycle = 0;

    public MockSingleMotor(String name) {
        super(name);
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        this.dutyCycle = dutyCycle;
    }

    @Override
    public double getDutyCycle() {
        return dutyCycle;
    }

}

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import frc.robot.drive.modes.Mode;
import frc.robot.drive.modes.Mode.Parameters;
import frc.robot.drive.modes.Mode.Type;
import frc.robot.interfaces.Drivebase;
import frc.robot.interfaces.Drivebase.DriveMotion;
import org.junit.jupiter.api.Test;
import org.strongback.components.Motor.ControlMode;
import org.strongback.mock.Mock;
import org.strongback.mock.MockMotor;

public class TestDrivebase {

    class MockDriveMode extends Mode {
        public String name;
        public int callCount = 0;
        public double leftPower = 0;
        public double rightPower = 0;

        public MockDriveMode(String name) {
            super(name, ControlMode.DutyCycle);
            this.name = name;
        }

        @Override
        public DriveMotion getMotion(double leftSpeed, double rightSpeed) {
            callCount++;
            return new DriveMotion(leftPower, rightPower);
        }

        @Override
        public void enable() {}

        @Override
        public void disable() {}

        @Override
        public boolean hasFinished() {
            return true;
        }

        @Override
        public String getName() {
            return "mock";
        }
    }

    @Test
    public void testDriveMode() {
        MockMotor leftMotor = Mock.stoppedMotor();
        MockMotor rightMotor = Mock.stoppedMotor();
        MockDriveMode arcade = new MockDriveMode("MockArcade");
        Drivebase drive = new DrivebaseImpl(leftMotor, rightMotor);
        // Register this drive mode so it can be used.
        drive.register(Type.ARCADE_DUTY_CYCLE, arcade);
        // Tell the drive subsystem to use it.
        drive.setMode(
                new Parameters(Type.ARCADE_DUTY_CYCLE));
        int expectedCallCount = 0;

        // Subsystems should start disabled, so shouldn't be calling the
        // DrivedriveMode.
        assertEquals(expectedCallCount, arcade.callCount);
        drive.execute(0);
        assertEquals(expectedCallCount, arcade.callCount);
        assertEquals(0, leftMotor.get(), 0.01);
        assertEquals(0, rightMotor.get(), 0.01);

        // Enable the drivebase
        arcade.leftPower = 0.5;
        arcade.rightPower = 0.75;
        drive.enable();
        drive.execute(0); // Should call getMotion() on driveMode.
        assertEquals(++expectedCallCount, arcade.callCount);
        // Check that the motors now have power.
        assertEquals(arcade.leftPower, leftMotor.get(), 0.01);
        assertEquals(arcade.rightPower, rightMotor.get(), 0.01);

        // Update the speed and see if the motors change.
        arcade.leftPower = -0.1;
        arcade.rightPower = 1;
        drive.execute(0); // Should call getMotion() on driveMode.
        assertEquals(++expectedCallCount, arcade.callCount);
        // Check that the motors now have power.
        assertEquals(arcade.leftPower, leftMotor.get(), 0.01);
        assertEquals(arcade.rightPower, rightMotor.get(), 0.01);

        // Change driveMode and see if the outputs are different
        MockDriveMode cheesy = new MockDriveMode("MockCheesy");
        cheesy.leftPower = 1;
        cheesy.rightPower = -1;
        drive.register(Type.CHEESY, cheesy);
        // Tell the drive subsystem to use it.
        drive.setMode(new Parameters(Type.CHEESY));
        drive.execute(0);
        assertEquals(1, cheesy.callCount); // first time running this driveMode
        assertEquals(cheesy.leftPower, leftMotor.get(), 0.01);
        assertEquals(cheesy.rightPower, rightMotor.get(), 0.01);

        // Disable and confirm that the driveMode isn't called and the motors are
        // stopped
        drive.disable();
        drive.execute(0); // Should no call getMotion() on driveMode.
        assertEquals(expectedCallCount, arcade.callCount);
        assertEquals(0, leftMotor.get(), 0.01);
        assertEquals(0, rightMotor.get(), 0.01);
    }

}

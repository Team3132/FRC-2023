package frc.robot.drive.modes;



import frc.robot.interfaces.Drivebase.DriveMotion;
import org.strongback.components.Motor.ControlMode;

/**
 * Tells the drive wheels to drive forward at a constant power level.
 */
public class ConstantDrive extends Mode {
    private DriveMotion motion = new DriveMotion(0, 0);

    public ConstantDrive(String name, ControlMode controlMode) {
        super(name, controlMode);
    }

    @Override
    public void reset(Parameters parameters) {
        motion = new DriveMotion(parameters.value, parameters.value);
    }

    @Override
    public DriveMotion getMotion(double leftSpeed, double rightSpeed) {
        return motion;
    }
}

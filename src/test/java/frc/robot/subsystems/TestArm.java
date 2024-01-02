package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.Config;
import org.junit.jupiter.api.Test;

public class TestArm {
    @Test
    void testCalculateCurrentHeight() {
        double extension = 0.2;
        // Arm straight up.
        assertEquals(
                Config.arm.telescope.pivotJointHeight
                        + Config.arm.telescope.baseLength + extension,
                ArmImpl.calculateCurrentHeight(90, extension), 0.1);
        // 45 degrees off vertical
        double height = Config.arm.telescope.pivotJointHeight
                + Math.sin(Math.toRadians(45))
                        * (Config.arm.telescope.baseLength + extension);
        assertEquals(height,
                ArmImpl.calculateCurrentHeight(90 - 45, extension), 0.1);
        assertEquals(height,
                ArmImpl.calculateCurrentHeight(90 + 45, extension), 0.1);
    }
}

package frc.robot.lib;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;
import org.strongback.mock.MockDoubleSupplier;

public class TestLowPassFilter {
    @Test
    public void testLowPassFilter() {
        MockDoubleSupplier source = new MockDoubleSupplier();
        source.setValue(10.0);

        LowPassFilter filter = new LowPassFilter(source, 0.2);

        assertEquals(10.0, filter.getAsDouble(), 0.001);

        source.setValue(0.0);

        assertEquals(10 * 0.8, filter.getAsDouble(), 0.001);
        assertEquals(10 * 0.8 * 0.8, filter.getAsDouble(), 0.001);
        assertEquals(10 * 0.8 * 0.8 * 0.8, filter.getAsDouble(), 0.001);
        assertEquals(10 * 0.8 * 0.8 * 0.8 * 0.8, filter.getAsDouble(), 0.001);
    }
}

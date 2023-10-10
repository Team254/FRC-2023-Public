package com.team254.frc2023.limelight.undistort;


import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;


public class InterpolatingUndistortMapTest {
    private class MockUndistortMap implements UndistortMap {

        @Override
        public double[] normalizedToUndistortedNormalized(double x, double y) {
            return normalizedToUndistortedNormalized(x * pixelWidth(), y * pixelHeight());
        }

        @Override
        public double[] pixelToUndistortedNormalized(int x, int y) {
            return new double[] { (double) x / pixelWidth(), (double) y / pixelHeight() };
        }

        @Override
        public int pixelWidth() {
            return 320;
        }

        @Override
        public int pixelHeight() {
            return 240;
        }
    }

    @Test
    public void testInterpolatingUndistortMap() {
        UndistortMap inner_map = new MockUndistortMap();
        InterpolatingUndisortMap map = new InterpolatingUndisortMap(1280, 960, inner_map);
        for (int x = 0; x < map.pixelWidth(); x++) {
            for (int y = 0; y < map.pixelHeight(); y++) {
                double[] out = map.pixelToUndistortedNormalized(x, y);
                double expectedX = ((x * 1.0) * inner_map.pixelWidth() / map.pixelWidth()) / inner_map.pixelWidth();
                double expectedY = ((y * 1.0) * inner_map.pixelWidth() / map.pixelWidth()) / inner_map.pixelHeight();
                Assertions.assertArrayEquals(out, new double[] { expectedX, expectedY}, 1.0e-9);
            }
        }
    }
}

package com.team254.frc2023.limelight.undistort;

public class InterpolatingUndisortMap implements UndistortMap {
    final int pixelWidth_;
    final int pixelHeight_;
    final UndistortMap map_;

    public InterpolatingUndisortMap(int pixelWidth, int pixelHeight, UndistortMap map) {
        this.pixelWidth_ = pixelWidth;
        this.pixelHeight_ = pixelHeight;
        this.map_ = map;
    }

    @Override
    public double[] normalizedToUndistortedNormalized(double x, double y) {
        return normalizedToUndistortedNormalized(x * pixelWidth(), y * pixelHeight());
    }

    @Override
    public double[] pixelToUndistortedNormalized(int x, int y) {
        // Find a floating point that represents the input coordinate in map resolution
        double mapResX = ((double) x) * map_.pixelWidth() / pixelWidth();
        double mapResY = ((double) y) * map_.pixelHeight() / pixelHeight();

        // Find how far between floor and ceil the coord is
        int floorX = (int) Math.floor(mapResX);
        int floorY = (int) Math.floor(mapResY);
        double dx = mapResX - floorX;
        double dy = mapResY - floorY;

        // Fetch 4 adjacent corners from map
        double[] x0_y0 = map_.pixelToUndistortedNormalized(floorX, floorY);
        double[] x0_y1 = map_.pixelToUndistortedNormalized(floorX, floorY+1);
        double[] x1_y0 = map_.pixelToUndistortedNormalized(floorX + 1, floorY);
        double[] x1_y1 = map_.pixelToUndistortedNormalized(floorX + 1, floorY + 1);

        // Perform bilinear interpolation using the surrounding data points for both x and y
        double[] output = new double[] { 0, 0 };
        for (int i = 0; i < 2; ++i) {
            output[i] = (1 - dx) * (1 - dy) * x0_y0[i]
                    + dx * (1 - dy) * x1_y0[i]
                    + (1 - dx) * dy * x0_y1[i]
                    + dx * dy * x1_y1[i];
        }
        return output;
    }

    @Override
    public int pixelWidth() {
        return pixelWidth_;
    }

    @Override
    public int pixelHeight() {
        return pixelHeight_;
    }
}

package com.team254.frc2023.limelight.undistort;

public interface UndistortMap {
    /**
     * Get an undistorted point with normalized input and output
     * @param x Normalized [0, 1] row value to undistort
     * @param y Normalized [0, 1] col value to undistort
     * @return Normalized + undistorted  point in form [x,y]
     */
    double[] normalizedToUndistortedNormalized(double x, double y);

    /**
     * Get an undistorted point given raw pixel space input, and returning normalized output
     * @param x Pixel [0, pixelHeight()] row value to undistort
     * @param y Normalized [0, pixelWidth()] col value to undistort
     * @return Normalized + undistorted  point in form [x,y]
     */
    double[] pixelToUndistortedNormalized(int x, int y);

    /**
     * Width of this map in number of pixels
     */
    int pixelWidth();

    /**
     * Height of this map in number of pixels
     */
    int pixelHeight();
}
